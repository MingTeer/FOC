/**
 * @file pid.h
 * @brief PID控制器头文件
 * @version 1.0
 * @date 2025-11-03
 *
 * 提供位置式PID控制器的实现，适用于FOC电机控制中的电流环、速度环和位置环控制
 */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含头文件 */
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID控制器结构体
 */
typedef struct {
    int32_t kp;            /* 比例系数 */
    int32_t ki;            /* 积分系数 */
    int32_t kd;            /* 微分系数 */
    int32_t target;        /* 目标值 */
    int32_t last_error;    /* 上一次误差 */
    int32_t integral;      /* 积分累计值 */
    int32_t output_max;    /* 输出上限 */
    int32_t output_min;    /* 输出下限 */
    int32_t integral_max;  /* 积分项上限 */
    int32_t integral_min;  /* 积分项下限 */
} PID;

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_max 输出上限
 * @param integral_max 积分项上限
 */
void PID_Init(PID *pid, int32_t kp, int32_t ki, int32_t kd, int32_t output_max, int32_t integral_max, int32_t target);

/**
 * @brief 增量式PID计算
 * @param pid PID控制器结构体指针
 * @param feedback 反馈值
 * @return PID输出值
 */
int32_t Incremental_PID_Calculate(PID *pid, int32_t feedback);

/**
 * @brief 位置式PID计算
 * @param pid PID控制器结构体指针
 * @param feedback 反馈值
 * @return PID输出值
 */
int32_t Position_PID_Calculate(PID *pid, int32_t feedback);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
