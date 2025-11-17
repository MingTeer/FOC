/**
 * @file pid.c
 * @brief PID控制器实现文件
 * @version 1.0
 * @date 2025-11-03
 *
 * 实现位置式PID控制器，适用于FOC电机控制
 */

#include "pid.h"

/**
 * @brief 初始化PID控制器（通用，支持增量式和位置式）
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_max 输出上限
 * @param output_min 输出下限
 */
void PID_Init(PID *pid, float kp, float ki, float kd, float max, float target)
{
    /* 设置PID参数 */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /* 设置输出限制 */
    pid->output_max = max;
    pid->output_min = -max;

    /* 初始化内部变量 */
    pid->target = target;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}

/**
 * @brief 增量式PID计算
 * @param pid PID控制器结构体指针
 * @param feedback 反馈值
 * @return PID输出值
 */
float Incremental_PID_Calculate(PID *pid, float feedback)
{
    float error = pid->target - feedback;

    // 计算误差变化
    float delta_error = error - pid->last_error;

    // 增量式积分项更新
    pid->integral += pid->ki * error;

    // 增量式PID增量计算
    float delta_output = pid->kp * delta_error + pid->integral + pid->kd * (delta_error - (pid->last_error - pid->integral));

    // 输出限制
    float output = delta_output;
    if(output > pid->output_max) {
        output = pid->output_max;
    } else if(output < pid->output_min) {
        output = pid->output_min;
    }

    pid->last_error = error;

    return output;
}

/**
 * @brief 位置式PID计算
 * @param pid PID控制器结构体指针
 * @param feedback 反馈值
 * @return PID输出值
 */
float Position_PID_Calculate(PID *pid, float feedback)
{
    float error = pid->target - feedback;

    // 积分项计算
    pid->integral += error;

    // 积分限幅
    float integral_max = pid->output_max / pid->ki;
    float integral_min = pid->output_min / pid->ki;
    if(pid->integral > integral_max) {
        pid->integral = integral_max;
    } else if(pid->integral < integral_min) {
        pid->integral = integral_min;
    }

    // 微分项计算
    float derivative = error - pid->last_error;

    // 位置式PID输出计算
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限制
    if(output > pid->output_max) {
        output = pid->output_max;
    } else if(output < pid->output_min) {
        output = pid->output_min;
    }

    pid->last_error = error;

    return output;
}
