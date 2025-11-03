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
    float kp;            /* 比例系数 */
    float ki;            /* 积分系数 */
    float kd;            /* 微分系数 */
    float target;        /* 目标值 */
    float last_error;    /* 上一次误差 */
    float integral;      /* 积分累计值 */
    float output_max;    /* 输出上限 */
    float output_min;    /* 输出下限 */
} PID_Controller;

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_max 输出上限
 * @param output_min 输出下限
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float max, float target);

/**
 * @brief 位置式PID计算
 * @param pid PID控制器结构体指针
 * @param target 目标值
 * @param feedback 反馈值
 * @return PID输出值
 */
float PID_Calculate(PID_Controller *pid, float target, float feedback);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
