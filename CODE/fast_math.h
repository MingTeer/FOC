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

#endif // __FAST_MATH_H
