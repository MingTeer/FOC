#include "current_sensor.h"
#include "tim.h"


/* 全局变量定义 */
uint16_t current_offset_a = 0;  /* A相电流零漂偏移 (ADC原始值) */
uint16_t current_offset_b = 0;  /* B相电流零漂偏移 (ADC原始值) */
 
uint16_t gain_factor = 1;  /* 放大系数，调节电流采样率 */


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
        /* 将电压偏移转换回ADC原始值 */
        current_offset_a = (uint16_t)(sum_a / samples * ADC_RESOLUTION / VREF_V);
        current_offset_b = (uint16_t)(sum_b / samples * ADC_RESOLUTION / VREF_V);
    }

    gain_factor = (uint16_t)(1 / CURRENT_GAIN / SHUNT_RESISTANCE);
}

/**
 * @brief  获取两相电流值
 * @param  currents: uint16_t 数组，currents[0]=A相电流，currents[1]=B相电流 (ADC原始值减去零漂)
 * @retval 无
 */
void Get_Phase_Currents(int16_t *currents)
{
    /* 从DMA缓冲区读取ADC原始值 */
    uint16_t raw_a = adc_dma_buffer[0];
    uint16_t raw_b = adc_dma_buffer[1];

    /* 减去零漂偏移，直接输出ADC值差值 */
    currents[0] = (raw_a - current_offset_a) / gain_factor;
    currents[1] = (raw_b - current_offset_b) / gain_factor;
}
