### DRV8833 电机驱动板

+ AIN1/AIN2是输入，连在树莓派GPIO上
+ STBY是**待机/使能脚**，作用 `SLP` 类似：**必须拉高，DRV8833 才会工作**。因此需要接到树莓派的3V3上
+ GND要同时与电源的'-'，树莓派的GND连接，即电源，驱动板，树莓派必须共地
+ VM与电源+连接：用一个母对公的杜邦线，电源夹子夹在杜邦线的公端上（电源''-''与驱动板GND的连接类似）
+ AO1/AO2是输出，用跳线加母对公杜邦线连接在电机的两个引脚上
+ NC是no connection，不用管



### STM32无法正常烧录的问题

烧录不成功，显示

```
**Target No device found,** 

**Error in initializing ST-Link device**

**Reason: no device found on target**
```

1.先检查接线有没有接对，尤其注意杜邦线接触是否良好

2.设置debugger

查看该链接的最后一个对话：https://chatgpt.com/share/6a462676-8100-83ec-989b-d72891c5a893

主要是

```
Port: SWD
Frequency: 100 kHz
Mode: Under Reset
Reset mode: Hardware reset
```

3.如果实在不行，去STM32CubeProgrammer reset：

```
解决方法是用 Connect under reset 救回来。

在 STM32CubeProgrammer 里设置：

Port: SWD
Frequency: 100 kHz
Mode: Under Reset
Reset mode: Hardware reset

然后按住板子上的 RESET，点 Connect，连接过程中松开 RESET。

如果能连上，直接：

Full chip erase

擦除之后它就能恢复正常烧录。
```



### 起手式：第一步是在SYS->Debug中设置Serial Wire

![img 2026-07-03 13.17.51](/Users/tangentii/Library/Application Support/typora-user-images/img 2026-07-03 13.17.51.jpg)





ctrl-s保存不了，必须点击保存图标



TIMER

注意设置每一个TIM自己的

+ Prescalar（对时钟信号进行分频，71表示72分频）

+ counter period（自动重装载的周期，存在一个自动重装载计时器当中，65535表示每间隔65536个周期将计时器重新置为0，即定时器从0数到65535之后再从0开始数）.

bug0：电机应该用6V，只用了5V了；

bug1：忘记修改TIMER3的counter period（默认65535，需要3599）

bug2:一开始的时候AI没有叫我们修改TIMER2的prescalar，用的默认值0，但是实际上应该变成71；



### 连接MaixCAM

https://chatgpt.com/share/6a47b1c8-a8bc-83ec-848f-74d462daca33



### 轮子不转的Debug

用万用表测量这一条轮子上（电源、电机、驱动板，stm32）各个点的电压，最终定位到发现stm32上对应的一个GPIO端口输出的电压就不对

然后再定位到代码，发现计时器设计的不对



### 基础小车四电机控制代码的分析

+ 2个定时器(TIM2/TIM3)
  + TIM2用于测量遥控器传来的PWM信号的脉冲宽度
    + TIM接在APB1上，APB1是72MHz，对于TIM2，要将Prescaler设置为71（即进行72分频），得到1Mhz，即每一个周期1us；
    + counter period保持最大为65535
  + TIM3用于向电机（驱动板）输出PWM脉冲
    + TIM3的prescaler = 0，即不分频保持72MHz; 但
    + 但是counter period设置为3599，目的是为了3600分频，得到的PWM信号的频率就是20kHz
    + 每一个周期叫做一个tick，一个PWM周期内就有3600个tick；
    + TIM3内部有计数器CNT和比较寄存器CCR，
    + ```__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);```就是将duty的值写入对应通道的CCR
    + 近似有当 CNT < CCR 时，PWM 输出高电平；当 CNT >= CCR 时，PWM 输出低电平

+ 遥控器的输出：PWM，占空比反应了摇杆的的位移

+ `HAL`: Hardware Abstraction Layer

+ `HAL_GetTick()`

  + 返回值是从 HAL 初始化完成以来，经过了多少毫秒。

  + 在 STM32 HAL 里，通常用 **SysTick 定时器** 每隔 `1 ms` 触发一次中断。

    每触发一次，HAL 内部的全局计数变量就加 1，HAL_GetTick()本质上是在读取这个毫秒计数器。



总的来说，

+ 四个GPIO接口使用【中断输入（上升沿与下降沿出发中断）】的方法接收来自遥控器的PWM信号；
+ 接收到PWM的上升沿/下降沿的时候，主循环中断，进入中断处理函数
  + 接收到上升沿，则说明之前是0，现在变成了1，则在中断处理函数中用当前的时间更新rc_start(全局变量)的值；
  + 接收到下降沿，则用当前时间减去rc_start，就可以得到高电平的宽度/PWM，写入rc_us（也是全局变量）
+ 主循环中则不断读取rc_us的值，然后调用motor_set()设置电机的速度



### 超声波测距

最开始使用阻塞性超声波传感器测距，轮子非常卡。修改为中断型的就好很多

HC-SR04 有四个引脚：

| 引脚 | 作用         |
| ---- | ------------ |
| VCC  | 5V 电源      |
| GND  | 地           |
| Trig | 触发测距输入 |
| Echo | 回波时间输出 |

测距时，单片机先给 **Trig 引脚一个至少 10 微秒的高电平脉冲**。HC-SR04 收到这个触发信号后，会自动发出一组 **40 kHz 的超声波脉冲**。

40 kHz 的意思是声波每秒振动 40000 次，超过人耳能听到的范围，所以叫超声波。

发出超声波后，HC-SR04 会把 **Echo 引脚拉高**。当超声波遇到前方物体并反射回来，被传感器接收到以后，Echo 引脚再被拉低。

所以 Echo 高电平持续的时间，就是超声波从传感器出发、到物体、再返回传感器的总时间。



代码实现的基本逻辑：

+ 维护状态机全局变量`us_state`
+ 在主循环中
  + 如果当前状态是`DONE`，表示刚刚接收到下降沿信号，则计算出距离，并将状态更新为空闲`IDLE`
  + 如果当前状态是`WAIT_RISE || WAIT_FALL`，表示已经发出trig信号，正在等待echo信号的上升沿下降沿，这时候什么都不需要做（可以做一些超时的fallback）
  + 如果当前状态是`IDLE`，并且又到了该发出Trig的时间，则发出Trig信号

+ 中断处理：stm32的GPIO接收到echo信号的上升沿或者下降沿的时候出发中断，进入中断处理函数；
  + 中断处理函数主要记录边沿到来的时间
