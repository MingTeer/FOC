#include "current_sensor.h"
#include "tim.h"


/* 全局变量定义 */
float current_offset_a = 0.0f;  /* A相电流零漂偏移 */
float current_offset_b = 0.0f;  /* B相电流零漂偏移 */

/**
 * @brief  零漂校准，读取1000次取平均值
 * @param  无
 * @retval 无
 */
void Current_Sensor_Calibrate_Zero(void)
{
    float sum_a = 0.0f, sum_b = 0.0f;
    uint16_t i;

    /* 累加1000次采样值 */
    for(i = 0; i < ZERO_CALIBRATION_SAMPLES; i++)
    {
        /* 等待一小段时间确保数据更新 */
        HAL_Delay(1);
        while (adc_dma_buffer[0] == 0 || adc_dma_buffer[1] == 0);
        
        /* Read raw ADC values and convert to voltage */
        float voltage_a = (float)adc_dma_buffer[0] * VREF_V / ADC_RESOLUTION;
        float voltage_b = (float)adc_dma_buffer[1] * VREF_V / ADC_RESOLUTION;

        /* Accumulate voltage samples */
        sum_a += voltage_a;
        sum_b += voltage_b;
    }

    /* 计算平均值作为零漂偏移 */
    float samples = (float)ZERO_CALIBRATION_SAMPLES;
    if (samples > 0.0f)
    {
        current_offset_a = sum_a / samples;
        current_offset_b = sum_b / samples;
    }
}

/**
 * @brief  获取两相电流值
 * @param  currents: 浮点数组，currents[0]=A相电流，currents[1]=B相电流 (A)
 * @retval 无
 */
void Get_Phase_Currents(float *currents)
{
    float voltage_a, voltage_b;

    /* 从DMA缓冲区读取ADC原始值 */
    uint16_t raw_a = adc_dma_buffer[0];
    uint16_t raw_b = adc_dma_buffer[1];

    /* Convert raw ADC readings to voltage (V) */
    voltage_a = (float)raw_a * VREF_V / ADC_RESOLUTION;
    voltage_b = (float)raw_b * VREF_V / ADC_RESOLUTION;

    /* 计算相对于中点电压的差值 */
    voltage_a = voltage_a - current_offset_a;
    voltage_b = voltage_b - current_offset_b;

    /* Convert voltage offset to current (A) */
    float sense_factor = CURRENT_GAIN * SHUNT_RESISTANCE;

    currents[0] = voltage_a / sense_factor;
    currents[1] = voltage_b / sense_factor;
}
