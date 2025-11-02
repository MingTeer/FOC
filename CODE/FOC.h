/**
 * @file    FOC.h
 * @brief   FOC(Field Oriented Control)电机控制算法头文件
 * @version 1.0
 * @date    2025
 * @note    实现FOC磁场定向控制算法，包括电压控制、角度计算和电流变换
 */

#ifndef __FOC_H
#define __FOC_H

/*
 * =================================================================================
 *                            系统初始化函数
 * =================================================================================
 */

/**
 * @brief  FOC系统初始化
 * @param  power_voltage   电源电压，单位V (典型值: 12V, 24V, 48V)
 * @param  limit_voltage   限制电压，单位V (建议不超过电源电压的80%)
 * @retval void
 * @note   初始化电压参数，执行传感器校准，设置安全限制
 *         调用此函数前需确保AS5600编码器已正常工作
 *         函数执行时间约2.5秒(包含传感器校准时间)
 */
void FOC_Init(float power_voltage, float limit_voltage);

/*
 * =================================================================================
 *                            FOC控制算法函数
 * =================================================================================
 */

/**
 * @brief  设置空间矢量PWM电压 (FOC核心算法)
 * @param  Uq             q轴电压分量，单位V (控制转矩)
 * @param  Ud             d轴电压分量，单位V (控制磁场，通常设为0)
 * @param  angle_eletric  电角度，单位弧度，范围[0, 2π)
 * @retval void
 * @note   实现反Park变换和空间矢量PWM算法
 *         输入dq坐标系电压，转换为三相PWM输出
 *         内部使用快速查表法计算三角函数，提高实时性
 *         电压会自动限制在安全范围内
 */
void setPhaseVoltage(float Uq, float Ud, float angle_eletric);

/*
 * =================================================================================
 *                            传感器相关函数
 * =================================================================================
 */

/**
 * @brief  获取当前电角度位置
 * @param  void
 * @retval 电角度，单位弧度，范围[0, 2π)
 * @note   结合AS5600编码器反馈和零点偏移计算实际电角度
 *         函数内部自动进行角度归一化处理
 *         用于FOC算法的坐标系变换
 */
float electricAngle_position(void);

/*
 * =================================================================================
 *                            数学工具函数
 * =================================================================================
 */

/**
 * @brief  角度归一化到[-180°, 180°)
 * @param  angle  输入角度，单位度
 * @retval 归一化后的角度，范围[-180°, 180°)
 * @note   便于角度差值计算和显示
 *         常用于角度误差计算和数据处理
 */
float normallizeDegree(float angle);

/**
 * @brief  Calculate dq-axis currents (Clarke + Park)
 * @param  current_a  Phase-A current (A)
 * @param  current_b  Phase-B current (A)
 * @param  angle_el   Electrical angle (rad)
 * @param  iq_out     Output pointer for q-axis current
 * @param  id_out     Output pointer for d-axis current
 */
void cal_Iq_Id(float current_a, float current_b, float angle_el, float *iq_out, float *id_out);

/**
 * @brief  相序自检：Ud>0 使磁场对齐 d 轴、Uq=0，无转矩抖动小
 * @param  void
 * @retval void
 * @note   用于验证FOC算法的相序正确性
 */
void PhaseOrderSelfTest(void);

#endif
