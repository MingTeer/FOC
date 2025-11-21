/**
 * @file    FOC.h
 * @brief   Integer-only FOC (Field Oriented Control) interfaces.
 */

#ifndef __FOC_H
#define __FOC_H

#include "stm32g0xx.h"

/* Fixed-point constants */
#define PI_Q15             32768   /* pi in Q1.15 */
#define TWO_PI_Q15         65536   /* 2*pi in Q1.15 */
#define ONE_DIV_SQRT3_Q15  18919   /* 1/sqrt(3) * 32768 */
#define TWO_DIV_SQRT3_Q15  37838   /* 2/sqrt(3) * 32768 */
#define SQRT3_Q15          56809   /* sqrt(3) * 32768 */

extern int16_t limit_voltage;        /* limit voltage (mV) */

/* System init */
void FOC_Init(uint16_t power_voltage_mv);

/* Core control helpers */
void setPhaseVoltage(int16_t Uq, int16_t Ud, int16_t angle_eletric);
void setPhaseVoltage_SVPWM(int16_t Uq, int16_t Ud, int16_t angle_el);
int16_t electricAngle_position(void);
int16_t normallizeDegree(int16_t angle);
void cal_Iq_Id(int16_t current_a, int16_t current_b, int16_t angle_el, int16_t *iq_out, int16_t *id_out);
void PhaseOrderSelfTest(void);

#endif
