#ifndef __AS5600_H
#define __AS5600_H
#include "stm32g0xx.h"                  // Device header

extern int16_t angle_q15;

int16_t Get_absolute_angle(void);

#endif
