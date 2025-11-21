#ifndef __CURRENT_SENSOR_H
#define __CURRENT_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"

/* 电流采样配置常量 */
#define CURRENT_GAIN          50.0f      /* 电流采样IC放大倍数 */
#define SHUNT_RESISTANCE      0.01f      /* 采样电阻 10毫欧 */
#define VREF_V                3.3f       /* ADC reference voltage (V) */
#define ADC_RESOLUTION        4096.0f    /* 12位ADC分辨率 */
#define ZERO_CALIBRATION_SAMPLES 1000U   /* 零漂校准采样次数 */

/* 函数声明 */
void Get_Phase_Currents(int16_t *currents);           /* 获取两相电流值 */
void Current_Sensor_Calibrate_Zero(void);           /* 零漂校准 */

#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_SENSOR_H */

