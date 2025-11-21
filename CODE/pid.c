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
 * @param integral_max 积分项上限
 */
void PID_Init(PID *pid, int32_t kp, int32_t ki, int32_t kd, int32_t output_max, int32_t integral_max, int32_t target)
{
    /* 设置PID参数 */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /* 设置输出限制 */
    pid->output_max = output_max;
    pid->output_min = -output_max;

    /* 设置积分项限制 */
    pid->integral_max = integral_max;
    pid->integral_min = -integral_max;

    /* 初始化内部变量 */
    pid->target = target;
    pid->last_error = 0;
    pid->integral = 0;
}

/**
 * @brief 增量式PID计算
 * @param pid PID控制器结构体指针
 * @param feedback 反馈值
 * @return PID输出值
 */
int32_t Incremental_PID_Calculate(PID *pid, int32_t feedback)
{
    int32_t error = pid->target - feedback;

    // 计算误差变化
    int32_t delta_error = error - pid->last_error;

    // 增量式积分项更新 (纯整数)
    pid->integral += ((int64_t)pid->ki * error);
    
    // 积分项限幅
    if(pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if(pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    // 增量式PID增量计算 (纯整数)
    int64_t delta_output = ((int64_t)pid->kp * delta_error) + 
                           pid->integral + 
                           ((int64_t)pid->kd * (delta_error - (error - pid->last_error)));

    // 输出限制
    int32_t output = (int32_t)delta_output;
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
int32_t Position_PID_Calculate(PID *pid, int32_t feedback)
{
    int32_t error = pid->target - feedback;

    // 积分项计算 (纯整数累加)
    pid->integral += error;

    // 积分项限幅
    if(pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if(pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    // 微分项计算
    int32_t derivative = error - pid->last_error;

    // 位置式PID输出计算 (纯整数运算)
    int64_t output = ((int64_t)pid->kp * error) + 
                     ((int64_t)pid->ki * pid->integral) + 
                     ((int64_t)pid->kd * derivative);

    // 输出限制
    int32_t result = (int32_t)output;
    if(result > pid->output_max) {
        result = pid->output_max;
    } else if(result < pid->output_min) {
        result = pid->output_min;
    }

    pid->last_error = error;

    return result;
}
