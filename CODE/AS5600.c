#include "AS5600.h"
#include "i2c.h"
#include "stm32g0xx.h"                  // Device header
#include "stdio.h" 

uint8_t data[2] = {0,0};
int16_t angle_q15 = 0;

int16_t Get_absolute_angle(void)
{

    uint8_t dev_addr = (0x36 << 1);  
    HAL_StatusTypeDef status = I2C1_ReadRegister_Normal(dev_addr, 0x0C, data, 2, 1);  
    if (status == HAL_OK)
    {
        uint16_t angle_data = (data[0] << 8) | data[1];
        // 12位(0-4095)映射到16位(0-65535)，即Q15格式的(-PI到PI)
        angle_q15 = (int16_t)(angle_data << 4);
    }
    return angle_q15;
}
