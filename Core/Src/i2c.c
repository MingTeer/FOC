/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "stdio.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00400627;  // 1MHz (Fast Mode Plus) for 64MHz PCLK1
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#define I2C_TIMEOUT_MS        1000U
#define I2C_POLL_TIMEOUT      10U

/**
 * @brief  I2C写函数（正常模式）
 * @param  DevAddress: 目标设备地址
 * @param  pData: 待发送数据缓冲区指针
 * @param  Size: 发送数据长度
 * @param  Timeout: 超时时间（ms）
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C1_Write_Normal(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status;

    // 启动正常模式发送
    status = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, pData, Size, Timeout);
    return status;
}

/**
 * @brief  I2C读函数（正常模式）
 * @param  DevAddress: 目标设备地址
 * @param  pData: 数据缓冲区指针
 * @param  Size: 接收数据长度
 * @param  Timeout: 超时时间（ms）
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C1_Read_Normal(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status;

    // 启动正常模式接收
    status = HAL_I2C_Master_Receive(&hi2c1, DevAddress, pData, Size, Timeout);
    return status;
}

/**
 * @brief  I2C寄存器读操作（先写寄存器地址再读，正常模式）
 * @param  DevAddress: 目标设备地址
 * @param  RegAddress: 寄存器地址
 * @param  pData: 数据缓冲区指针
 * @param  Size: 数据长度
 * @param  Timeout: 超时时间（ms）
 * @retval HAL状态
 */
HAL_StatusTypeDef I2C1_ReadRegister_Normal(uint8_t DevAddress, uint8_t RegAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status;

    // 先使用正常模式发送寄存器地址
    status = I2C1_Write_Normal(DevAddress, &RegAddress, 1, Timeout);
    if (status != HAL_OK)
    {
        return status;
    }

    // 再进行正常模式接收
    status = I2C1_Read_Normal(DevAddress, pData, Size, Timeout);
    return status;
}

/**
 * @brief  检测I2C设备是否连接
 * @param  DevAddress: 设备地址（8位格式）
 * @retval 1=设备存在, 0=设备不存在
 */
uint8_t I2C_IsDeviceConnected(uint8_t DevAddress)
{
    // 尝试向设备地址写一个字节（不实际发送数据）
    if (HAL_I2C_Master_Transmit(&hi2c1, DevAddress, NULL, 0, 100) == HAL_OK)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  扫描I2C总线上的所有设备
 */
void I2C_ScanDevices(void)
{
    printf("Scanning I2C bus...\r\n");
    for (uint8_t addr = 1; addr < 128; addr++)
    {
        uint8_t dev_addr = (addr << 1);  // 转换为8位格式
        if (I2C_IsDeviceConnected(dev_addr))
        {
            printf("Found device at 0x%02X (7-bit)\r\n", addr);
        }
    }
    printf("Scan complete.\r\n");
}

/* USER CODE END 1 */
