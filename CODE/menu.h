/**
 * @file    menu.h
 * @brief   USART菜单系统和模式选择头文件
 * @version 1.0
 * @date    2025
 * @note    实现USART菜单显示、模式选择和目标值读取功能
 */

#ifndef __MENU_H
#define __MENU_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 菜单缓冲区大小定义 */
#define MENU_RX_BUFFER_SIZE     64
#define MENU_CMD_MAX_LEN        32

/* 控制模式枚举 */
typedef enum
{
    MENU_MODE_IDLE = 0,          // 空闲模式
    MENU_MODE_SPEED,             // 速度控制模式
    MENU_MODE_TORQUE,            // 转矩控制模式
    MENU_MODE_POSITION,          // 位置控制模式
    MENU_MODE_CURRENT,           // 电流控制模式
    MENU_MODE_MAX
} menu_mode_t;

/* 菜单状态结构体 */
typedef struct
{
    menu_mode_t current_mode;        // 当前模式
    float target_value;              // 目标值
    bool mode_selected;              // 模式是否已选择
    bool waiting_for_value;          // 是否等待输入目标值
    uint8_t rx_buffer[MENU_RX_BUFFER_SIZE];  // DMA接收缓冲区
    uint16_t rx_length;              // 接收到的数据长度
    bool rx_ready;                   // 接收完成标志
} menu_state_t;

/* 函数声明 */
void Menu_Init(void);
void Menu_Show(void);
void Menu_Process(void);
void Menu_StartDMAReceive(void);
menu_mode_t Menu_GetMode(void);
float Menu_GetTargetValue(void);
bool Menu_IsModeSelected(void);

/* DMA接收完成回调函数（在usart.c中实现） */
void Menu_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* UART IDLE中断回调函数（在stm32g0xx_it.c中实现） */
void Menu_UART_IdleCallback(UART_HandleTypeDef *huart);

#endif /* __MENU_H */

