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
#include "fast_math.h"
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
  int16_t iq;
  int16_t id;
  int16_t radian;
  int16_t rpm;   
} motor_state_t;

static volatile motor_state_t motor_state_isr = {0};
menu_mode_t current_control_mode = MENU_MODE_IDLE;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t tim6_duration_us = 0;  // TIM6中断执行时间 (微秒)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
PID PID_Speed;
PID PID_I;
PID PID_Id;

int16_t current_I[2] = {0,0};
int16_t iq_temp = 0;
int16_t id_temp = 0;
int16_t uq = 0;
int16_t ud = 0;
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
  FOC_Init(12000);
	MX_TIM3_Init(500);
	MX_TIM6_Init(100);
  Menu_Init();
  /* USER CODE END 2 */
  PID_Init(&PID_Speed,500,8,0,100000,100000,0);
  PID_Init(&PID_I,300,8,0,limit_voltage,limit_voltage / 16,0);
  PID_Init(&PID_Id,200,16,0,limit_voltage,limit_voltage / 16,0);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    Menu_Process();
//    
//    // // 根据菜单模式设置目标值
//    if(Menu_IsModeSelected())
//    {
//      current_control_mode = Menu_GetMode();
//      float target = Menu_GetTargetValue();
//      
//      if(current_control_mode == MENU_MODE_SPEED)
//      {
//        // 速度控制模式：设置速度目标值
//        if(target > 1300)target = 1300;
//        PID_Speed.target = target;
//      }
//      else if(current_control_mode == MENU_MODE_TORQUE)
//      {
//        // 转矩控制模式：直接设置iq目标值
//        PID_I.target = target;
//      }
//    }
    printf("%d,%d\n", motor_state_isr.rpm, motor_state_isr.iq);
    
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
    uint32_t start_cycles = __HAL_TIM_GET_COUNTER(&htim6);
    
    int16_t electrical_angle = motor_state_isr.radian;
    Get_Phase_Currents(current_I);
    cal_Iq_Id(current_I[0], current_I[1], electrical_angle, &iq_temp, &id_temp);
    motor_state_isr.iq = (iq_temp + motor_state_isr.iq * 9) / 10;
    motor_state_isr.id = (id_temp + motor_state_isr.id * 9) / 10;
    uq = Position_PID_Calculate(&PID_I,motor_state_isr.iq);
    ud = Position_PID_Calculate(&PID_Id,motor_state_isr.id);
    setPhaseVoltage_SVPWM(uq, ud, electrical_angle);
    
    uint32_t end_cycles = __HAL_TIM_GET_COUNTER(&htim6);
    if(end_cycles >= start_cycles)
    {
      tim6_duration_us = end_cycles - start_cycles;
    }
    else
    {
      tim6_duration_us = (htim6.Init.Period + 1 - start_cycles) + end_cycles;
    }
    /* USER CODE END TIM6_IRQ */
  }
  else if(htim->Instance == TIM3)
  {
    int16_t radian_value = -electricAngle_position();
    motor_state_isr.radian = radian_value;
    
    static int16_t last_angle = 0;
    static uint8_t angle_initialized = 0;
    static int32_t rpm_filtered_q4 = 0; // Q4格式以保留滤波精度

    if(!angle_initialized)
    {
      last_angle = radian_value;
      angle_initialized = 1;
      motor_state_isr.rpm = 0;
    }
    else
    {
      int16_t angle_diff = radian_value - last_angle;
      int32_t rpm_raw = ((int32_t)angle_diff * 67) >> 8;
      rpm_filtered_q4 = (rpm_filtered_q4 * 9 + (rpm_raw << 4)) / 10;
      motor_state_isr.rpm = (int16_t)(rpm_filtered_q4 >> 4);

      // 速度环处理
//     if(current_control_mode == MENU_MODE_SPEED)
//     {
        // 注意：假设 Position_PID_Calculate 返回整型
        // 如果原代码是浮点除法 /1000.0f，这里保留整数除法可能需要调整增益
        PID_I.target = Position_PID_Calculate(&PID_Speed,motor_state_isr.rpm) / 1000;
//     }

      last_angle = radian_value;
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
