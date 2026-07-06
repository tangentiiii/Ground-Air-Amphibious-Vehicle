"""MaixCAM Pro: raw wireless video + browser-side detection overlays."""

import _thread
import json
import socket
import time

from maix import app, camera, display, http, image, nn


MODEL_PATH = "/root/models/yolo11n.mud"
MODEL_TYPE = "yolo11"  # yolo11, yolov8, or yolov5
CONFIDENCE_THRESHOLD = 0.50
IOU_THRESHOLD = 0.45

# Camera capture and network settings.  Stream resolution/FPS can then be
# changed live from the browser, up to the capture resolution.
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
DEFAULT_STREAM_WIDTH = 640
DEFAULT_STREAM_HEIGHT = 480
DEFAULT_STREAM_FPS = 15
VIDEO_PORT = 8000
CONTROL_PORT = 8001
SHOW_ON_DEVICE = True

# Camera mounted 90 degrees anti-clockwise -> correct by 270 degrees
# anti-clockwise (= 90 degrees clockwise). Use 0 for normal mounting.
FRAME_ROTATION_DEGREES = 90


PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MaixCAM Pro 目标检测</title><style>
body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
h2{margin:14px 6px 8px}.controls{margin:8px}label{margin:0 8px}
#view{position:relative;display:inline-block;line-height:0;max-width:96vw}
#video{display:block;max-width:96vw;max-height:78vh;object-fit:contain}
#overlay{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
#state{color:#aaa;font-size:14px;margin:8px}
</style></head><body>
<h2>MaixCAM Pro 实时目标检测</h2>
<div class="controls">
 <label>分辨率 <select id="resolution">
  <option value="320x240">320×240</option>
  <option value="640x480" selected>640×480</option>
 </select></label>
 <label>帧率 <select id="fps">
  <option>5</option><option>10</option><option selected>15</option>
  <option>20</option><option>25</option><option>30</option>
 </select> FPS</label>
</div>
<div id="view"><img id="video" src="/stream"><canvas id="overlay"></canvas></div>
<div id="state">正在连接检测数据…</div>
<script>
const video=document.getElementById('video'), canvas=document.getElementById('overlay');
const ctx=canvas.getContext('2d'), state=document.getElementById('state');
let objects=[],dpr=window.devicePixelRatio||1;
function resizeCanvas(){
 dpr=window.devicePixelRatio||1;
 const w=video.clientWidth,h=video.clientHeight;
 const pixelW=Math.max(1,Math.round(w*dpr)),pixelH=Math.max(1,Math.round(h*dpr));
 if(canvas.width!==pixelW||canvas.height!==pixelH){canvas.width=pixelW;canvas.height=pixelH}
 draw();
}
function draw(){
 const width=video.clientWidth,height=video.clientHeight;
 // Draw in CSS pixels while the backing bitmap uses physical screen pixels.
 ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,width,height);
 ctx.strokeStyle='#ff2020';ctx.fillStyle='#ff2020';ctx.lineWidth=2;
 ctx.font='600 16px -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif';
 ctx.textBaseline='alphabetic';
 for(const o of objects){
  const x=o.x*width,y=o.y*height,w=o.w*width,h=o.h*height;
  ctx.strokeRect(x,y,w,h);const text=o.label+' '+o.score.toFixed(2);
  const ty=Math.max(16,y-4),tw=ctx.measureText(text).width+6;
  ctx.fillRect(x,ty-16,tw,19);ctx.fillStyle='#fff';ctx.fillText(text,x+3,ty);
  ctx.fillStyle='#ff2020';
 }
}
new ResizeObserver(resizeCanvas).observe(video);video.onload=resizeCanvas;
const events=new EventSource('http://'+location.hostname+':8001/events');
events.onmessage=e=>{const d=JSON.parse(e.data);objects=d.objects;state.textContent=
 '检测目标 '+objects.length+' 个 · '+d.width+'×'+d.height+' · '+d.fps+' FPS';draw()};
events.onerror=()=>{state.textContent='检测数据连接中断，正在重连…'};
async function configure(){
 const [w,h]=document.getElementById('resolution').value.split('x');
 const fps=document.getElementById('fps').value;
 await fetch('http://'+location.hostname+':8001/config?width='+w+'&height='+h+'&fps='+fps,
             {method:'POST'});
}
document.getElementById('resolution').onchange=configure;
document.getElementById('fps').onchange=configure;
</script></body></html>"""


stream_config = {
    "width": DEFAULT_STREAM_WIDTH,
    "height": DEFAULT_STREAM_HEIGHT,
    "fps": DEFAULT_STREAM_FPS,
}
latest_metadata = json.dumps({"objects": [], **stream_config})
metadata_sequence = 0


def create_detector():
    classes = {"yolo11": nn.YOLO11, "yolov8": nn.YOLOv8, "yolov5": nn.YOLOv5}
    kind = MODEL_TYPE.lower()
    if kind not in classes:
        raise ValueError("MODEL_TYPE must be yolo11, yolov8, or yolov5")
    return classes[kind](model=MODEL_PATH, dual_buff=True)


def orient_frame(frame):
    angle = FRAME_ROTATION_DEGREES % 360
    if angle not in (0, 90, 180, 270):
        raise ValueError("FRAME_ROTATION_DEGREES must be 0, 90, 180, or 270")
    return frame.rotate(angle) if angle else frame


def contain_transform(src_w, src_h, dst_w, dst_h):
    scale = min(dst_w / src_w, dst_h / src_h)
    return scale, (dst_w - src_w * scale) / 2, (dst_h - src_h * scale) / 2


def map_objects(objects, detector, src_w, src_h, model_w, model_h, out_w, out_h):
    """Map model letterbox coordinates to normalized stream coordinates."""
    model_scale, model_pad_x, model_pad_y = contain_transform(
        src_w, src_h, model_w, model_h
    )
    out_scale, out_pad_x, out_pad_y = contain_transform(src_w, src_h, out_w, out_h)
    result = []
    for obj in objects:
        # Undo model letterboxing, then apply stream letterboxing.
        sx = (obj.x - model_pad_x) / model_scale
        sy = (obj.y - model_pad_y) / model_scale
        sw = obj.w / model_scale
        sh = obj.h / model_scale
        x = (sx * out_scale + out_pad_x) / out_w
        y = (sy * out_scale + out_pad_y) / out_h
        w = sw * out_scale / out_w
        h = sh * out_scale / out_h
        # Clip boxes to the visible stream image.
        x2, y2 = min(1.0, x + w), min(1.0, y + h)
        x, y = max(0.0, x), max(0.0, y)
        if x2 > x and y2 > y:
            result.append({
                "x": x, "y": y, "w": x2 - x, "h": y2 - y,
                "label": detector.labels[obj.class_id], "score": obj.score,
            })
    return result


def parse_query(target):
    values = {}
    if "?" not in target:
        return values
    for item in target.split("?", 1)[1].split("&"):
        if "=" in item:
            key, value = item.split("=", 1)
            values[key] = value
    return values


def send_response(conn, status, body=b"", content_type="application/json"):
    headers = (
        "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
        "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n"
        % (status, content_type, len(body))
    )
    conn.sendall(headers.encode() + body)


def handle_control_client(conn):
    global metadata_sequence
    try:
        request = conn.recv(2048).decode("utf-8", "ignore")
        first = request.split("\r\n", 1)[0].split()
        if len(first) < 2:
            return
        method, target = first[0], first[1]
        if target.startswith("/events"):
            conn.sendall(
                b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
                b"Cache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n"
                b"Connection: keep-alive\r\n\r\n"
            )
            sent = -1
            while True:
                if sent != metadata_sequence:
                    conn.sendall(("data: " + latest_metadata + "\n\n").encode())
                    sent = metadata_sequence
                time.sleep(0.03)
        elif target.startswith("/config") and method == "POST":
            query = parse_query(target)
            width = int(query.get("width", stream_config["width"]))
            height = int(query.get("height", stream_config["height"]))
            fps = int(query.get("fps", stream_config["fps"]))
            allowed = ((320, 240), (640, 480))
            if (width, height) not in allowed or not 1 <= fps <= 30:
                send_response(conn, "400 Bad Request", b'{"error":"invalid config"}')
            else:
                stream_config.update(width=width, height=height, fps=fps)
                send_response(conn, "200 OK", json.dumps(stream_config).encode())
        else:
            send_response(conn, "404 Not Found", b'{"error":"not found"}')
    except Exception:
        pass
    finally:
        conn.close()


def control_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("", CONTROL_PORT))
    server.listen(4)
    while True:
        conn, _ = server.accept()
        _thread.start_new_thread(handle_control_client, (conn,))


def publish_metadata(mapped_objects, width, height, fps):
    global latest_metadata, metadata_sequence
    latest_metadata = json.dumps({
        "objects": mapped_objects, "width": width, "height": height, "fps": fps
    })
    metadata_sequence += 1


def main():
    detector = create_detector()
    model_w, model_h = detector.input_width(), detector.input_height()
    cam = camera.Camera(CAPTURE_WIDTH, CAPTURE_HEIGHT, detector.input_format())
    screen = display.Display() if SHOW_ON_DEVICE else None

    streamer = http.JpegStreamer(host="", port=VIDEO_PORT, client_number=4)
    streamer.set_html(PAGE)
    streamer.start()
    _thread.start_new_thread(control_server, ())
    print("Open http://<MaixCAM-IP>:%d/ on your computer" % VIDEO_PORT)

    next_stream_time = 0.0
    try:
        while not app.need_exit():
            source = orient_frame(cam.read())
            inference = source.resize(model_w, model_h, fit=image.Fit.FIT_CONTAIN,
                                      method=image.ResizeMethod.BILINEAR)
            objects = detector.detect(inference, conf_th=CONFIDENCE_THRESHOLD,
                                      iou_th=IOU_THRESHOLD)

            now = time.monotonic()
            fps = stream_config["fps"]
            if now >= next_stream_time:
                out_w, out_h = stream_config["width"], stream_config["height"]
                stream_frame = source.resize(out_w, out_h, fit=image.Fit.FIT_CONTAIN,
                                             method=image.ResizeMethod.BILINEAR)
                mapped = map_objects(objects, detector, source.width(), source.height(),
                                     model_w, model_h, out_w, out_h)
                streamer.write(stream_frame)  # Raw frame: no boxes are drawn here.
                publish_metadata(mapped, out_w, out_h, fps)
                if screen is not None:
                    screen.show(stream_frame)
                next_stream_time = now + 1.0 / fps
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()
