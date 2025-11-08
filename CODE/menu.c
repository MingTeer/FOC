/**
 * @file    menu.c
 * @brief   USART菜单系统及模式选择实现
 * @version 1.0
 * @date    2025
 */

#include "menu.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 全局菜单状态 */
static menu_state_t menu_state;

/* 控制模式名称（全局显示用） */
static const char* mode_names[] = {
    "空闲",
    "速度控制",
    "转矩控制",
    "位置控制",
    "电流控制"
};

/* 控制模式单位（全局显示用） */
static const char* mode_units[] = {
    "",
    "转/分",
    "牛·米",
    "位置",
    "安"
};

/**
 * @brief  菜单系统初始化
 * @retval 无
 */
void Menu_Init(void)
{
    memset(&menu_state, 0, sizeof(menu_state_t));
    menu_state.current_mode = MENU_MODE_IDLE;
    menu_state.target_value = 0.0f;
    menu_state.mode_selected = false;
    menu_state.waiting_for_value = false;
    menu_state.rx_ready = false;
    
    /* 启动DMA接收 */
    Menu_StartDMAReceive();
    
    /* 显示初始化菜单 */
    HAL_Delay(100);
    Menu_Show();
}

/**
 * @brief  显示菜单
 * @retval 无
 */
void Menu_Show(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("         FOC电机控制菜单\r\n");
    printf("========================================\r\n");
    
    if (menu_state.mode_selected && menu_state.current_mode != MENU_MODE_IDLE)
    {
        printf("当前模式: %s\r\n", mode_names[menu_state.current_mode]);
        printf("目标数值: %.2f %s\r\n", menu_state.target_value, mode_units[menu_state.current_mode]);
    }
    else
    {
        printf("当前模式: %s\r\n", mode_names[menu_state.current_mode]);
    }
    
    printf("\r\n");
    printf("请选择控制模式:\r\n");
    printf("  1 - 速度控制 (单位: 转/分)\r\n");
    printf("  2 - 转矩控制 (单位: 牛·米)\r\n");
    printf("  3 - 位置控制 (单位: 位置)\r\n");
    printf("  4 - 电流控制 (单位: 安)\r\n");
    printf("\r\n");
    
    if (menu_state.waiting_for_value)
    {
        printf("请输入目标数值 (q退出): ");
    }
    else
    {
        printf("请输入模式序号 (0-4): ");
    }
    
    fflush(stdout);
}

/**
 * @brief  解析命令字符串
 * @param  cmd_str 命令字符串
 * @retval 无
 */
static void Menu_ParseCommand(const char* cmd_str)
{
    if (cmd_str == NULL || strlen(cmd_str) == 0)
    {
        return;
    }
    
    /* 去除前后空格和换行符 */
    char trimmed[MENU_CMD_MAX_LEN];
    int i = 0, j = 0;
    int len = strlen(cmd_str);
    
    /* 跳过前导空白字符 */
    while (i < len && (cmd_str[i] == ' ' || cmd_str[i] == '\t' || cmd_str[i] == '\r' || cmd_str[i] == '\n'))
    {
        i++;
    }
    
    /* 复制有效字符（不含CRLF） */
    while (i < len && j < MENU_CMD_MAX_LEN - 1)
    {
        if (cmd_str[i] == '\r' || cmd_str[i] == '\n')
        {
            break;
        }
        trimmed[j++] = cmd_str[i++];
    }
    trimmed[j] = '\0';
    
    /* 去除尾部空白 */
    while (j > 0 && (trimmed[j-1] == ' ' || trimmed[j-1] == '\t'))
    {
        trimmed[--j] = '\0';
    }
    
    if (strlen(trimmed) == 0)
    {
        return;
    }
    
    /* 是否处于等待目标数值输入 */
    if (menu_state.waiting_for_value)
    {
        /* 检查是否为退出命令 'q' 或 'Q' */
        if ((trimmed[0] == 'q' || trimmed[0] == 'Q') && trimmed[1] == '\0')
        {
            /* 退出当前模式，返回主菜单 */
            menu_state.waiting_for_value = false;
            menu_state.mode_selected = false;
            menu_state.current_mode = MENU_MODE_IDLE;
            menu_state.target_value = 0.0f;
            
            printf("\r\n");
            printf("已退出，返回主菜单\r\n");
            Menu_Show();
            return;
        }
        
        /* 尝试解析为浮点数 */
        char* endptr;
        float value = strtof(trimmed, &endptr);
        
        /* 检查是否为有效的数值 */
        if (endptr != trimmed && *endptr == '\0')
        {
            /* 更新目标值，保持在当前模式 */
            menu_state.target_value = value;
            menu_state.mode_selected = true;
            
            printf("\r\n");
            printf("目标数值已更新: %.2f %s\r\n", menu_state.target_value, mode_units[menu_state.current_mode]);
            printf("当前模式: %s\r\n", mode_names[menu_state.current_mode]);
            printf("可继续输入新目标值，或输入 q 退出\r\n");
            printf("请输入目标数值 (q退出): ");
            fflush(stdout);
        }
        else
        {
            printf("\r\n");
            printf("无效输入，请输入数字 (q退出)\r\n");
            printf("请输入目标数值 (q退出): ");
            fflush(stdout);
        }
        
        return;
    }
    
    /* 解析模式选择命令 */
    char *endptr = NULL;
    long parsed_mode = strtol(trimmed, &endptr, 10);
    if (endptr == trimmed || *endptr != '\0')
    {
        printf("\r\n");
        printf("输入无效! 请输入0-4\r\n");
        Menu_Show();
        return;
    }
    int mode_num = (int)parsed_mode;
    
    /* 处理返回命令 */
    if (mode_num == 0)
    {
        menu_state.current_mode = MENU_MODE_IDLE;
        menu_state.mode_selected = false;
        menu_state.waiting_for_value = false;
        menu_state.target_value = 0.0f;
        
        printf("\r\n");
        printf("已切换到空闲模式\r\n");
        Menu_Show();
        return;
    }
    
    /* 选择控制模式 (1-4) */
    if (mode_num >= 1 && mode_num < MENU_MODE_MAX)
    {
        menu_state.current_mode = (menu_mode_t)mode_num;
        menu_state.mode_selected = false;
        menu_state.target_value = 0.0f;
        
        printf("\r\n");
        printf("已选择模式: %s\r\n", mode_names[menu_state.current_mode]);
        
        menu_state.waiting_for_value = true;
        printf("请输入目标数值 (单位: %s, q退出): ", mode_units[menu_state.current_mode]);
        fflush(stdout);
    }
    else
    {
        printf("\r\n");
        printf("无效输入，请输入0-4\r\n");
        Menu_Show();
    }
}

/**
 * @brief  处理接收到的数据
 * @retval 无
 */
void Menu_Process(void)
{
    if (menu_state.rx_ready)
    {
        /* 确保接收到的字符串以\0结尾 */
        uint16_t len = menu_state.rx_length;
        if (len >= MENU_RX_BUFFER_SIZE)
        {
            len = MENU_RX_BUFFER_SIZE - 1;
        }
        menu_state.rx_buffer[len] = '\0';
        
        /* 解析命令 */
        Menu_ParseCommand((const char*)menu_state.rx_buffer);
        
        /* 清除接收标志 */
        menu_state.rx_ready = false;
        menu_state.rx_length = 0;
        
        /* 重新启动DMA收数据 */
        Menu_StartDMAReceive();
    }
}

/**
 * @brief  启动DMA接收
 * @retval 无
 */
void Menu_StartDMAReceive(void)
{
    /* 清空接收缓冲区 */
    memset(menu_state.rx_buffer, 0, MENU_RX_BUFFER_SIZE);
    
    /* 启动DMA接收 */
    if (HAL_UART_Receive_DMA(&huart1, menu_state.rx_buffer, MENU_RX_BUFFER_SIZE) != HAL_OK)
    {
        /* 启动异常，可在此添加错误提示 */
        return;
    }
    
    /* 启用UART IDLE中断检测接收结束 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/**
 * @brief  获取当前控制模式
 * @retval 当前控制模式
 */
menu_mode_t Menu_GetMode(void)
{
    return menu_state.current_mode;
}

/**
 * @brief  获取目标数值
 * @retval 目标数值
 */
float Menu_GetTargetValue(void)
{
    return menu_state.target_value;
}

/**
 * @brief  判断模式是否已选择
 * @retval true-模式已选择, false-未选择
 */
bool Menu_IsModeSelected(void)
{
    return menu_state.mode_selected;
}

/**
 * @brief  UART DMA接收完成回调
 * @param  huart UART句柄
 * @retval 无
 * @note   在usart.c文件中调用
 */
void Menu_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 计算实际接收长度 */
        menu_state.rx_length = MENU_RX_BUFFER_SIZE - huart->RxXferCount;
        
        /* 标记已接收完成 */
        menu_state.rx_ready = true;
    }
}

/**
 * @brief  UART IDLE中断回调
 * @param  huart UART句柄
 * @retval 无
 * @note   在stm32g0xx_it.c调用，用于检测DMA接收结束
 */
void Menu_UART_IdleCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t received_len = MENU_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        
        if (received_len > 0 && received_len < MENU_RX_BUFFER_SIZE)
        {
            /* 有数据且缓冲区未满，表示一次接收完成 */
            menu_state.rx_length = received_len;
            menu_state.rx_ready = true;
            
            /* 停止DMA接收 */
            HAL_UART_DMAStop(huart);
        }
    }
}
