/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim3;

/* TIM1 初始化函数 */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0}; // 主模式配置结构体
  TIM_OC_InitTypeDef sConfigOC = {0};          // 输出比较配置结构体
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0}; // 死区和断保护结构体

  /* USER CODE BEGIN TIM1_Init 1 */
  __HAL_RCC_TIM1_CLK_ENABLE(); // 确保配置前打开TIM1时钟
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1; // 选择定时器1
  htim1.Init.Prescaler = 3; // 预分频器为7，(8分频后10kHz@80MHz)
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1; // 向中计数模式
  htim1.Init.Period = 999; // 自动重装载值，PWM周期(1000计数，对应10kHz@8MHz)
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟不分频
  htim1.Init.RepetitionCounter = 0; // 重复计数器
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // 不启用预装载
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK) // 基本输出比较初始化
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // 主模式触发输出选择
  // TRGO2 由CH4的高电平中点（PWM2）产生脉冲，用于触发ADC/DMA采样
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // 禁止主从模式
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) // 主从同步配置
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1; // 输出比较模式设为PWM1
  sConfigOC.Pulse = 0; // 初始脉冲宽度为0
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出极性为高
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // 互补输出极性为高
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; // 不使能快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; // 空闲状态为低
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补空闲为低
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) // 配置通道1
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) // 配置通道2
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) // 配置通道3
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1 ; // CH4在周期中点高电平，用作采样触发基准
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) // 通道4用于ADC触发
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE; // 运行时关闭输出失能
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE; // 空闲时关闭输出失能
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF; // 不加锁
  sBreakDeadTimeConfig.DeadTime = 0; // 死区时间为0
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE; // 不使能刹车功能
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH; // 刹车电平为高
  sBreakDeadTimeConfig.BreakFilter = 0; // 刹车滤波为0
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT; // 刹车模式输入
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE; // 不使能刹车2
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH; // 刹车2极性高
  sBreakDeadTimeConfig.Break2Filter = 0; // 刹车2滤波为0
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT; // 刹车2模式输入
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; // 禁止自动输出
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) // 死区与刹车配置
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1); // GPIO复用及输出管脚配置

}

/* TIM6 初始化函数 - 微秒中断定时器 */
void MX_TIM6_Init(uint16_t xus)
{
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6; // 选择定时器6
  htim6.Init.Prescaler = 63; // 64MHz/64=1MHz(1us一次计数)
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数模式
  htim6.Init.Period = xus - 1; // xus微秒中断周期
  htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟不分频
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // 不使用预装载
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) // 基本定时器初始化
  {
    Error_Handler();
  }

  // 启用TIM6更新中断
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/* TIM3 初始化函数 */
void MX_TIM3_Init(uint16_t xus)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0}; // 时钟源配置结构体
  TIM_MasterConfigTypeDef sMasterConfig = {0}; // 主模式配置结构体

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3; // 选择定时器3
  htim3.Init.Prescaler = 63; // 64MHz/64=1MHz(1us一次计数)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数模式
  htim3.Init.Period = xus - 1; // xus微秒中断周期
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟不分频
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // 不使用预装载
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) // 基本定时器初始化
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // 定时器内部时钟
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) // 时钟源配置
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // 主模式触发输出选择
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // 禁止主从模式
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) // 主从同步配置
  {
    Error_Handler();
  }

  // 启用TIM3更新中断
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0); // 设置较低优先级
    HAL_NVIC_EnableIRQ(TIM3_IRQn);         // 启用TIM3中断
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0); // 设置最高优先级
    HAL_NVIC_EnableIRQ(TIM6_IRQn);         // 启用TIM6中断
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PB3     ------> TIM1_CH2
    PB6     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}


void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/**
 * @brief  Set PWM duty cycle for PA8 (TIM1_CH1)
 * @param  duty_cycle: Duty cycle in percentage (0-100)
 * @retval None
 */
void TIM1_CH1_SetDutyCycle(uint32_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle);
}

/**
 * @brief  Set PWM duty cycle for PB3 (TIM1_CH2)
 * @param  duty_cycle: Duty cycle in percentage (0-100)
 * @retval None
 */
void TIM1_CH2_SetDutyCycle(uint32_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle);
}

/**
 * @brief  Set PWM duty cycle for PB6 (TIM1_CH3)
 * @param  duty_cycle: Duty cycle in percentage (0-100)
 * @retval None
 */
void TIM1_CH3_SetDutyCycle(uint32_t duty_cycle)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_cycle);
}

/**
 * @brief  Start PWM on all three channels
 * @retval None
 */
void TIM1_StartPWM(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
		TIM1_CH1_SetDutyCycle(0);
		TIM1_CH2_SetDutyCycle(0);
		TIM1_CH3_SetDutyCycle(0);
}

/**
 * @brief  Stop PWM on all three channels
 * @retval None
 */
void TIM1_StopPWM(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
}

/* USER CODE END 1 */
