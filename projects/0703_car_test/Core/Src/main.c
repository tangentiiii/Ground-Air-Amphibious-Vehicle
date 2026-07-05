/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

#define PWM_MAX 3599

//遥控器相关参数
#define RC_CH_COUNT 4 //遥控器通道数量
#define RC_MID_US 1500 //居中时遥控器输出的PWM信号的高电平宽度，单位us
#define RC_RANGE_US 500 //1500+-500是正常的范围
#define RC_MIN_VALID_US 900
#define RC_MAX_VALID_US 2100 //<900或者>2100的高电平信号视为噪声
#define RC_FAILSAFE_MS 120  //超过120ms没有收到有效遥控信号则停车；这叫作failsafe

// 默认：CH1 转向，CH2 油门
// 如果你用 CH3 当油门，把 THROTTLE_CH 改成 2
#define STEERING_CH 0
#define THROTTLE_CH 1

#define DEADBAND 0.06f // <6%的摇杆偏移视作0
#define EXPO_A 0.45f
#define MAX_SPEED 0.99f //调节最大速度（1.0为最大速度）
#define SLEW_STEP 0.015f //变化率限制，速度每一轮循环最多变化多少，避免电机极大的加速度
#define MIN_EFFECTIVE_CMD 0.12f //最小的有效占空比（占空比太小有可能推不动电机）

// region 不同接线下的方向调试
#define THROTTLE_REVERSE 0
#define STEERING_REVERSE 0

#define FL_REVERSE 1
#define RL_REVERSE 0
#define FR_REVERSE 0
#define RR_REVERSE 0
//endregion


//rc_us: 保存四个遥控通道的脉宽
//注意volatile的使用，此变量可能在主循环之外被修改，例如在中断函数修改，因此需要告诉编译器不要对这个变量进行优化
//对于rc_us: 会在EXT1中断回调里更新，主循环中只作读取
volatile uint16_t rc_us[RC_CH_COUNT] = {
  RC_MID_US, RC_MID_US, RC_MID_US, RC_MID_US
};

//记录每一个通道上升沿的时间
volatile uint16_t rc_start[RC_CH_COUNT] = {0, 0, 0, 0};
//记录每一个通道最后一次收到有效信号的时间，用于failsafe
volatile uint32_t rc_last_ms[RC_CH_COUNT] = {0, 0, 0, 0};

//debug
volatile uint32_t rc_edge_count[RC_CH_COUNT] = {0,0,0,0}; //查看终端出发的数量
volatile uint16_t rc_raw_width[RC_CH_COUNT] = {0,0,0,0}; //查看测出来的原始脉宽是多少

//static关键字：其他文件不能通过extern访问
static float left_now = 0.0f; //当前左轮的速度
static float right_now = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

//数学处理部分
static float abs_f(float x);
static float clamp_f(float x, float min_v, float max_v);
//遥控器输入处理
static float rc_to_norm(uint16_t pulse_us);
static float apply_deadband(float x);
static float apply_expo(float x);
static float apply_reverse(float x, int reverse);
static float slew_limit(float target, float current, float step);
//电机输出控制
static void motor_set(float cmd,
                      GPIO_TypeDef *in1_port, uint16_t in1_pin,
                      GPIO_TypeDef *in2_port, uint16_t in2_pin,
                      uint32_t pwm_channel,
                      int reverse);

static void motors_set(float left, float right);
static void motors_stop(void);
//遥控器中断测量
static void rc_update_from_exti(uint8_t ch_index,
                                GPIO_TypeDef *port,
                                uint16_t pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); //HAL：Hardware Abstraction Layer,硬件抽象层

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); //配置系统时钟

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); //初始化GPIO，把在.ioc中设置的各个GPIO设置好为对应的
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2); //启动TIM2的基础计数功能，TIM2开始按照指定的频率递增CNT寄存器

  //让TIM3开始输出PWM信号（占空比用的是在ioc中设置的初始值(Pulse)）
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // 两个 TB6612FNG 的 STBY 都接 PA8
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  motors_stop();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t throttle_us, steering_us;
    uint32_t throttle_last, steering_last;
    uint32_t now_ms = HAL_GetTick();  //得到当前时间；HAL_GetTick()的返回值，表示初始化完成以来，经过了多少毫秒

    //因为rc_us和rc_last_ms在中断时更新，为了避免主循环在读取时被中断打断，先关闭中断再读取这些数据
    __disable_irq(); //关闭中断
    throttle_us = rc_us[THROTTLE_CH];
    steering_us = rc_us[STEERING_CH];
    throttle_last = rc_last_ms[THROTTLE_CH];
    steering_last = rc_last_ms[STEERING_CH];
    __enable_irq(); //打开中断

    //失控保护
    if ((now_ms - throttle_last > RC_FAILSAFE_MS) ||
        (now_ms - steering_last > RC_FAILSAFE_MS))
    {
    	//一旦一段时间内没有收到遥控器的信号，则停车，但不退出主循环
      left_now = 0.0f;
      right_now = 0.0f;
      motors_stop();
      HAL_Delay(10);
      continue;
    }

    //遥控器脉冲宽度的处理
    float throttle = rc_to_norm(throttle_us);
    float steering = rc_to_norm(steering_us);

    throttle = apply_reverse(throttle, THROTTLE_REVERSE);
    steering = apply_reverse(steering, STEERING_REVERSE);

    throttle = apply_deadband(throttle);
    steering = apply_deadband(steering);

    throttle = apply_expo(throttle);
    steering = apply_expo(steering);

    float left_target = throttle + steering;
    float right_target = throttle - steering;

    //将left/right_target限制在-1~1内
    float max_mag = abs_f(left_target);
    if (abs_f(right_target) > max_mag)
    {
      max_mag = abs_f(right_target);
    }

    if (max_mag < 1.0f)
    {
      max_mag = 1.0f;
    }

    left_target /= max_mag;
    right_target /= max_mag;

    left_target *= MAX_SPEED;
    right_target *= MAX_SPEED;

    //限制速度的变化率
    left_now = slew_limit(left_target, left_now, SLEW_STEP);
    right_now = slew_limit(right_target, right_now, SLEW_STEP);

    //将信号输出到电机
    motors_set(left_now, right_now);

    //每轮结束后阻塞10ms
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  //遥控器的输入
  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; //外部中断输入；并且时上升沿和下降沿都会触发
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15*/
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static float abs_f(float x)
{
  return x >= 0.0f ? x : -x;
}

static float clamp_f(float x, float min_v, float max_v)
{
  if (x < min_v) return min_v;
  if (x > max_v) return max_v;
  return x;
}

static float rc_to_norm(uint16_t pulse_us)
{
  float x = ((float)pulse_us - (float)RC_MID_US) / (float)RC_RANGE_US;
  return clamp_f(x, -1.0f, 1.0f);
}

static float apply_deadband(float x)
{
  if (abs_f(x) < DEADBAND)
  {
    return 0.0f;
  }

  if (x > 0.0f)
  {
    return (x - DEADBAND) / (1.0f - DEADBAND);
  }
  else
  {
    return (x + DEADBAND) / (1.0f - DEADBAND);
  }
}

static float apply_expo(float x)
{
  return EXPO_A * x * x * x + (1.0f - EXPO_A) * x;
}

static float apply_reverse(float x, int reverse)
{
  return reverse ? -x : x;
}

static float slew_limit(float target, float current, float step)
{
  if (target > current + step)
  {
    return current + step;
  }
  else if (target < current - step)
  {
    return current - step;
  }
  else
  {
    return target;
  }
}

static void motor_set(float cmd,
                      GPIO_TypeDef *in1_port, uint16_t in1_pin,
                      GPIO_TypeDef *in2_port, uint16_t in2_pin,
                      uint32_t pwm_channel,
                      int reverse)
{
  if (reverse)
  {
    cmd = -cmd;
  }

  cmd = clamp_f(cmd, -1.0f, 1.0f);

  if (abs_f(cmd) < 0.001f)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, 0);
    return;
  }

  float mag = abs_f(cmd);

  mag = MIN_EFFECTIVE_CMD + mag * (1.0f - MIN_EFFECTIVE_CMD);
  if (mag > 1.0f)
  {
    mag = 1.0f;
  }

  uint16_t duty = (uint16_t)(mag * PWM_MAX + 0.5f);

  if (cmd > 0.0f)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
  }

  __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, duty);
}

static void motors_set(float left, float right)
{
  // 左前：PA6 PWM, PB10/PB11 方向
  motor_set(left,
            GPIOB, GPIO_PIN_10,
            GPIOB, GPIO_PIN_11,
            TIM_CHANNEL_1,
            FL_REVERSE);

  // 左后：PA7 PWM, PB12/PB13 方向
  motor_set(left,
            GPIOB, GPIO_PIN_12,
            GPIOB, GPIO_PIN_13,
            TIM_CHANNEL_2,
            RL_REVERSE);

  // 右前：PB0 PWM, PB14/PB15 方向
  motor_set(right,
            GPIOB, GPIO_PIN_14,
            GPIOB, GPIO_PIN_15,
            TIM_CHANNEL_3,
            FR_REVERSE);

  // 右后：PB1 PWM, PA4/PA5 方向
  motor_set(right,
            GPIOA, GPIO_PIN_4,
            GPIOA, GPIO_PIN_5,
            TIM_CHANNEL_4,
            RR_REVERSE);
}

static void motors_stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

static void rc_update_from_exti(uint8_t ch_index,
                                GPIO_TypeDef *port,
                                uint16_t pin)
{
	//刚刚来了一个GPIO的中断输入，然后进入这个函数
  if (ch_index >= RC_CH_COUNT)
  {
    return;
  }

  //debug：每次调用rc_update_from_exti的时候，应该是一个上升沿或者下降沿来临的时候
  	rc_edge_count[ch_index]++;//更新边沿数量；

  uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) //现在这个port是高电平，说明刚刚来的是一个上升沿
  {
    rc_start[ch_index] = now; //记录上升沿开始的时间
  }
  else //刚刚来的是一个下降沿
  {
    uint16_t width = (uint16_t)(now - rc_start[ch_index]); //统计上一个高电平脉冲宽度

    //debug
    rc_raw_width[ch_index] = width;

    if (width >= RC_MIN_VALID_US && width <= RC_MAX_VALID_US) //得到了一个有效的高电平脉冲宽度
    {
      rc_us[ch_index] = width; //记录
      rc_last_ms[ch_index] = HAL_GetTick();
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    rc_update_from_exti(0, GPIOA, GPIO_PIN_0);
  }
  else if (GPIO_Pin == GPIO_PIN_1)
  {
    rc_update_from_exti(1, GPIOA, GPIO_PIN_1);
  }
  else if (GPIO_Pin == GPIO_PIN_2)
  {
    rc_update_from_exti(2, GPIOA, GPIO_PIN_2);
  }
  else if (GPIO_Pin == GPIO_PIN_3)
  {
    rc_update_from_exti(3, GPIOA, GPIO_PIN_3);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
