/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "menu.h"
#include "AS5600.h"
#include "current_sensor.h"
#include "FOC.h"
#include "dma.h"
#include "pid.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct
{
  float iq;
  float id;
  float radian;
  int16_t rpm;    // 改为有符号整数，支持正反转方向（正表示正转，负表示反转）
} motor_state_t;

static volatile motor_state_t motor_state_isr = {0};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
PID_Controller PID_Speed;
PID_Controller PID_I;
PID_Controller PID_Id;
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  TIM1_StartPWM();
  HAL_Delay(1000);
  ADC_Start_DMA();
  Current_Sensor_Calibrate_Zero();
  FOC_Init(12.0f, 6.0f);
	MX_TIM3_Init(5);
	MX_TIM6_Init(500);
  Menu_Init();
  /* USER CODE END 2 */
  PID_Init(&PID_Speed,0.001,0.0001,0,6.0f,500);
  PID_Init(&PID_I,20,2,0,6.0f,0.1f);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 处理菜单命令 */
    Menu_Process();
    
    /* 根据选择的模式执行相应控制 */
    if (Menu_IsModeSelected())
    {
      menu_mode_t mode = Menu_GetMode();
      float target = Menu_GetTargetValue();
      
      /* 这里可以根据模式执行相应的控制逻辑 */
      /* 例如：根据mode和target更新PID控制器的目标值 */
      /* 根据mode和target更新PID控制器的目标值 */
      (void)mode;  /* 避免未使用变量警告 */
      (void)target; /* 避免未使用变量警告 */
    }
    
    HAL_Delay(10);  /* 避免CPU占用过高 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6)
  {
    // TIM6微秒中断处理程序 - 用户自定义功能
    /* USER CODE BEGIN TIM6_IRQ */
    float electrical_angle = motor_state_isr.radian;
    float current_I[2] = {0,0};
    Get_Phase_Currents(current_I);
    float iq_temp = 0.0f;
    float id_temp = 0.0f;
    cal_Iq_Id(current_I[0], current_I[1], electrical_angle, &iq_temp, &id_temp);
    float iq_value = iq_temp * 0.1 + motor_state_isr.iq * 0.9;
    float id_value = id_temp * 0.01 + motor_state_isr.id * 0.99;
    motor_state_isr.iq = iq_value;
    motor_state_isr.id = id_value;
    float uq = PID_Calculate(&PID_I,motor_state_isr.iq);
    float ud = PID_Calculate(&PID_Id,motor_state_isr.id);
    setPhaseVoltage(uq, ud, motor_state_isr.radian);
    /* USER CODE END TIM6_IRQ */
  }
  else if(htim->Instance == TIM3)
  {
    float radian_value = -electricAngle_position();
    motor_state_isr.radian = radian_value;
    static float last_angle = 0;
    static uint8_t angle_initialized = 0;
    static float rpm_filtered = 0;

    // 获取当前角度
    float current_angle = angle;

    if(!angle_initialized)
    {
      last_angle = current_angle;
      angle_initialized = 1;
      motor_state_isr.rpm = 0;
    }
    else
    {
      float angle_diff = current_angle - last_angle;

      // 计算角度差，处理0-360度跳变
      if(angle_diff > 180) {
          angle_diff -= 360;  // 正向跳变，减360
      } else if(angle_diff < -180) {
          angle_diff += 360;  // 反向跳变，加360
      }

      // 根据角度差符号确定旋转方向并计算转速
      float rpm_raw = angle_diff / 360.0f * 2000.0f * 60.0f;  // 2kHz采样, 转换为rpm

      // 低通滤波器平滑转速输出
      rpm_filtered = 0.9f * rpm_filtered + 0.1f * rpm_raw;

      // 存储带方向的转速（正表示正转，负表示反转）
      motor_state_isr.rpm = (int16_t)rpm_filtered;

      // float u = -PID_Calculate(&PID_Speed,motor_state_isr.rpm);
      // setPhaseVoltage(u, 0, motor_state_isr.radian);

      last_angle = current_angle;
    }
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

#ifdef  USE_FULL_ASSERT
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
