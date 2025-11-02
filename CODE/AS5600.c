#include "AS5600.h"
#include "i2c.h"
#include "stm32g0xx.h"                  // Device header
#include "stdio.h" 

uint8_t data[2] = {0,0};
float angle = 0;

float Get_absolute_angle(void)
{

    uint8_t dev_addr = (0x36 << 1);  
    HAL_StatusTypeDef status = I2C1_ReadRegister_Normal(dev_addr, 0x0C, data, 2, 10);  
    uint16_t angle_data = (data[0] << 8) | data[1];
    angle = (angle_data * 360.0f / 4096.0f);
    return -angle;
}
