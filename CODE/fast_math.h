#ifndef __FAST_MATH_H
#define __FAST_MATH_H

#include "stm32g0xx.h"                  // Device header

// 结构体，用于同时返回 sin 和 cos 的值
typedef struct {
    int16_t hSin;
    int16_t hCos;
} Trig_Components;

/**
  * @brief  使用查表法计算给定角度（Q1.15格式）的正弦和余弦值
  * @param  hAngle: 角度，使用Q1.15格式（-32768代表-PI, 32767代表+PI）
  * @retval Trig_Components 结构体，包含sin和cos结果（Q1.15格式）
  */
Trig_Components MCM_Trig_Functions(int16_t hAngle);

/**
 * @brief  纯整数快速 atan2，和 MCM_Trig_Functions 用同一角度格式
 * @param  y, x : 通常是 Q1.15 或其它定点格式的向量分量
 * @retval Q1.15 角度，范围约为 [-π, π) （即 [-32768, 32767]）
 */
int16_t MCM_Fast_Atan2_Q15(int16_t y, int16_t x);

#endif // __FAST_MATH_H
