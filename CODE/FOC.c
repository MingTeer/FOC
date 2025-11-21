#include "stm32g0xx.h"                  // Device header
#include "FOC.h"
#include "tim.h"
#include "AS5600.h"
#include "fast_math.h"
#include "stdio.h"

#define clip(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))

// 全局变量定义
int16_t power_voltage = 0;        // 电源电压 (mV)
int16_t limit_voltage = 0;        // 限制电压 (mV)
int16_t zero_electric_angle = 0;  // 零点电角度偏移 (Q15)
int pp = 7;                     // 电机极对数

static uint32_t voltage_to_duty_scale = 0; // 电压转占空比缩放因子 (Q16)
// Integer square root (non-restoring method) for 32-bit values
static inline uint32_t isqrt_u32(uint32_t x)
{
    uint32_t op = x;
    uint32_t res = 0;
    uint32_t one = 1u << 30;
    while (one > op)
    {
        one >>= 2;
    }
    while (one != 0)
    {
        if (op >= res + one)
        {
            op -= res + one;
            res = res + 2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return res;
}

/*
 * =================================================================================
 *                            角度处理函数
 * =================================================================================
 */

int16_t normallizeDegree(int16_t angle)
{
    int16_t normalized = angle % 360;
    if (normalized < -180)
    {
        normalized += 360;
    }
    else if (normalized >= 180)
    {
        normalized -= 360;
    }
    return normalized;
}

/*
 * =================================================================================
 *                            传感器相关函数
 * =================================================================================
 */

/**
 * @brief  获取当前电角度位置
 * @param  void
 * @retval 电角度，单位弧度，范围[0, 2PI)
 * @note   综合编码器读数和零点偏移计算实际电角度
 */
int16_t electricAngle_position(void)
{
    return (int16_t)(Get_absolute_angle() * pp) - zero_electric_angle;
}

/**
 * @brief  传感器校准流程
 * @param  void
 * @retval void
 * @note   通过外加转矩使电机到达已知位置，确保电角度零点校准
 */
void Check_Sensor(void)
{
    const int16_t align_voltage = limit_voltage / 2;
    const uint32_t settle_delay_ms = 800;
    const uint16_t sample_count = 64;

    setPhaseVoltage(0, align_voltage, 0);
    HAL_Delay(settle_delay_ms);

    int32_t sin_sum = 0;
    int32_t cos_sum = 0;
    for (uint16_t i = 0; i < sample_count; i++)
    {
        int16_t mechanical_angle = Get_absolute_angle();
        int16_t electrical_angle = (int16_t)(mechanical_angle * pp);
        Trig_Components trig = MCM_Trig_Functions(electrical_angle);
        sin_sum += trig.hSin;
        cos_sum += trig.hCos;
        HAL_Delay(5);
    }

    int16_t sin_avg = (int16_t)(sin_sum / sample_count);
    int16_t cos_avg = (int16_t)(cos_sum / sample_count);
    zero_electric_angle = MCM_Fast_Atan2_Q15(sin_avg, cos_avg);

    setPhaseVoltage(0, 0, 0);
    HAL_Delay(200);
}


/*
 * =================================================================================
 *                            FOC控制算法函数
 * =================================================================================
 */

/**
 * @brief  设置三相电压输出
 * @param  ua  A相电压，单位V
 * @param  ub  B相电压，单位V
 * @param  uc  C相电压，单位V
 * @note   将电压转换为PWM占空比，限制在供电电压范围内
 */
void set_power_voltage(int16_t ua,int16_t ub,int16_t uc)
{
    // 电压限制在允许范围内 (mV)
    ua = clip(ua, 0, power_voltage);
    ub = clip(ub, 0, power_voltage);
    uc = clip(uc, 0, power_voltage);

    // 转换为PWM占空比 (0-1000)
    // 使用定点乘法: (mV * scale) >> 16
    uint16_t duty_a = (uint16_t)((ua * voltage_to_duty_scale) >> 16);
    uint16_t duty_b = (uint16_t)((ub * voltage_to_duty_scale) >> 16);
    uint16_t duty_c = (uint16_t)((uc * voltage_to_duty_scale) >> 16);

    // 设置PWM占空比
    TIM1_CH1_SetDutyCycle(duty_a);
    TIM1_CH2_SetDutyCycle(duty_b);
    TIM1_CH3_SetDutyCycle(duty_c);
}

/**
 * @brief  设置空间矢量PWM输出 (FOC核心算法)
 * @param  Uq            q轴电压分量，单位V
 * @param  Ud            d轴电压分量，单位V
 * @param  angle_eletric 电角度，单位弧度
 * @note   实现Park变换和空间矢量PWM算法
 */
void setPhaseVoltage(int16_t Uq, int16_t Ud, int16_t angle_eletric)
{
    // 1. 使用查表法快速计算sin与cos
    Trig_Components components = MCM_Trig_Functions(angle_eletric);
    int16_t st = components.hSin;
    int16_t ct = components.hCos;

    // 2. 限制电压幅值
    int32_t Uq_abs = (Uq < 0) ? -Uq : Uq;
    int32_t Ud_abs = (Ud < 0) ? -Ud : Ud;
    uint32_t Uout_sq = (uint32_t)(Uq_abs * Uq_abs + Ud_abs * Ud_abs);
    uint32_t limit_sq = (uint32_t)limit_voltage * (uint32_t)limit_voltage;
    
    if (Uout_sq > limit_sq && Uout_sq != 0u)
    {
        uint32_t Uout = isqrt_u32(Uout_sq);
        int32_t scale_q15 = (int32_t)(((int64_t)limit_voltage << 15) / Uout);
        Uq = (int16_t)(((int32_t)Uq * scale_q15) >> 15);
        Ud = (int16_t)(((int32_t)Ud * scale_q15) >> 15);
    }

    // 3. Park变换 (反Park)
    // Ualpha = Ud * cos - Uq * sin
    // Ubeta  = Ud * sin + Uq * cos
    int32_t Ualpha = ((int32_t)Ud * ct - (int32_t)Uq * st) >> 15;
    int32_t Ubeta  = ((int32_t)Ud * st + (int32_t)Uq * ct) >> 15;

    // 4. Clarke变换 (反Clarke)
    // Ua = Ualpha + Vdc/2
    // Ub = (-0.5 * Ualpha + sqrt(3)/2 * Ubeta) + Vdc/2
    // Uc = (-0.5 * Ualpha - sqrt(3)/2 * Ubeta) + Vdc/2
    
    #define SQRT3_2_Q15 28377 // sqrt(3)/2 * 32768
    
    int16_t Vdc_half = power_voltage / 2;
    
    int16_t Ua = (int16_t)Ualpha + Vdc_half;
    int16_t Ub = (int16_t)((-(Ualpha >> 1) + ((Ubeta * SQRT3_2_Q15) >> 15))) + Vdc_half;
    int16_t Uc = (int16_t)((-(Ualpha >> 1) - ((Ubeta * SQRT3_2_Q15) >> 15))) + Vdc_half;

    // 5. 设置三相电压 (转换为mV)
    set_power_voltage(Ua, Ub, Uc);
}

/**
 * @brief  使用Sector-Voltage-PWM技术设置三相电压 (FOC核心算法)
 * @param  Uq            q轴电压分量，单位mV
 * @param  Ud            d轴电压分量，单位mV
 * @param  angle_el      电角度，单位弧度
 * @note   实现Sector-Voltage-PWM算法，适用于直流母线电压为正弦波的情况
 *         内部使用Q15定点数运算，提高执行效率
 */
void setPhaseVoltage_SVPWM(int16_t Uq, int16_t Ud, int16_t angle_el)
{
    // 1. 限制电压幅值 (圆形限幅，保持电压矢量方向)
    int32_t Uq_32 = Uq;
    int32_t Ud_32 = Ud;
    uint32_t mag_sq = (uint32_t)(Uq_32 * Uq_32 + Ud_32 * Ud_32);
    uint32_t limit_sq = (uint32_t)limit_voltage * (uint32_t)limit_voltage;

    // 只有当电压超过限制时才进行开方和缩放运算，正常闭环控制时很少触发
    if (mag_sq > limit_sq && mag_sq > 0)
    {
        uint32_t mag = isqrt_u32(mag_sq);
        // scale = limit / mag (Q15格式: limit << 15 / mag)
        int32_t scale = ((int32_t)limit_voltage << 15) / mag;
        Uq_32 = (Uq_32 * scale) >> 15;
        Ud_32 = (Ud_32 * scale) >> 15;
    }

    // 2. 反Park变换 (dq -> alpha, beta)
    Trig_Components trig = MCM_Trig_Functions(angle_el);
    int16_t st = trig.hSin;
    int16_t ct = trig.hCos;

    // Ualpha = Ud * cos - Uq * sin
    // Ubeta  = Ud * sin + Uq * cos
    int32_t Ualpha = (Ud_32 * ct - Uq_32 * st) >> 15;
    int32_t Ubeta  = (Ud_32 * st + Uq_32 * ct) >> 15;

    // 3. 反Clarke变换 (alpha, beta -> a, b, c 纯交流量)
    // Ua = Ualpha
    // Ub = -0.5 * Ualpha + sqrt(3)/2 * Ubeta
    // Uc = -0.5 * Ualpha - sqrt(3)/2 * Ubeta
    #define SQRT3_2_Q15 28377 

    int32_t Ua_ac = Ualpha;
    int32_t tmp = (Ubeta * SQRT3_2_Q15) >> 15;
    int32_t Ub_ac = -(Ualpha >> 1) + tmp;
    int32_t Uc_ac = -(Ualpha >> 1) - tmp;

    // 4. 零序分量注入 (SVPWM核心，中点平移法)
    // 寻找中间变量 (Min, Max)
    int32_t v_min = Ua_ac, v_max = Ua_ac;
    
    if (Ub_ac < v_min) v_min = Ub_ac;
    else if (Ub_ac > v_max) v_max = Ub_ac;
    
    if (Uc_ac < v_min) v_min = Uc_ac;
    else if (Uc_ac > v_max) v_max = Uc_ac;

    // 零序电压 v_zero = -0.5 * (max + min)
    int32_t v_zero = -(v_max + v_min) >> 1;

    // 5. 直接计算占空比并写入寄存器 (性能优化)
    // 目标: Duty = (U_ac + v_zero + Vdc/2) * scale >> 16
    // 优化: Duty = ((U_ac + v_zero) * scale >> 16) + (ARR / 2)
    // 假设 ARR = 1000 (根据 voltage_to_duty_scale 推算)
    
    int32_t duty_offset_a = ((Ua_ac + v_zero) * voltage_to_duty_scale) >> 16;
    int32_t duty_offset_b = ((Ub_ac + v_zero) * voltage_to_duty_scale) >> 16;
    int32_t duty_offset_c = ((Uc_ac + v_zero) * voltage_to_duty_scale) >> 16;
    
    // 中心对齐 (ARR / 2 = 500)
    const int32_t center_duty = 500;
    
    TIM1->CCR1 = (uint16_t)(center_duty + duty_offset_a);
    TIM1->CCR2 = (uint16_t)(center_duty + duty_offset_b);
    TIM1->CCR3 = (uint16_t)(center_duty + duty_offset_c);
}

/**
 * @brief  计算q轴和d轴电流分量（Iq和Id）
 * @param  current_a  A相电流（单位：A）
 * @param  current_b  B相电流（单位：A）
 * @param  angle_el   电角度（单位：弧度，通常为实际转子电角度）
 * @param  iq_out     q轴电流分量输出指针（单位：A）
 * @param  id_out     d轴电流分量输出指针（单位：A）
 * @retval void
 * @note   用于根据两相采样电流计算q轴和d轴电流，实现Clarke变换和Park变换
 */
void cal_Iq_Id(int16_t current_a, int16_t current_b, int16_t angle_el, int16_t *iq_out, int16_t *id_out)
{
    // Clarke变换 - 使用Q15定点数格式
    int16_t I_alpha = current_a;  // α轴电流直接等于A相电流
    
    // 优化: I_beta = (I_a + 2*I_b) / sqrt(3)
    // ONE_DIV_SQRT3_Q15 = 18919 (1/sqrt(3) in Q15)
    int32_t temp_sum = (int32_t)current_a + ((int32_t)current_b << 1);
    int16_t I_beta = (int16_t)((temp_sum * ONE_DIV_SQRT3_Q15) >> 15);

    // 使用查表法求 sin, cos
    Trig_Components components = MCM_Trig_Functions(angle_el);
    int16_t st = components.hSin;  // sin值，Q15格式
    int16_t ct = components.hCos;  // cos值，Q15格式

    // Park变换：计算dq轴电流
    int16_t I_q = ((int16_t)((int32_t)I_beta * ct >> 15)) - 
                  ((int16_t)((int32_t)I_alpha * st >> 15));
    int16_t I_d = ((int16_t)((int32_t)I_alpha * ct >> 15)) + 
                  ((int16_t)((int32_t)I_beta * st >> 15));

    // 输出结果
    if (iq_out != NULL)
    {
        *iq_out = I_q;
    }
    if (id_out != NULL)
    {
        *id_out = I_d;
    }
}

/*
 * =================================================================================
 *                            系统初始化函数
 * =================================================================================
 */

/**
 * @brief  FOC系统初始化
 * @param  p_voltage  电源电压，单位V
 * @retval void
 * @note   初始化电压参数，执行传感器校准，设置安全限制
 */
void FOC_Init(uint16_t p_voltage_mv)
{
    // 设置电源电压 (mV)
    power_voltage = (int16_t)p_voltage_mv;
    if (power_voltage > 0)
    {
        // 1000 * 65536 = 65536000
        voltage_to_duty_scale = 65536000 / power_voltage;
    }
    else
    {
        voltage_to_duty_scale = 0;
    }
    limit_voltage = (int16_t)(((int32_t)p_voltage_mv * 18919) >> 15);
    // 执行传感器校准
    Check_Sensor();
}

// 仅做自检：Ud>0 使磁场对齐 d 轴、Uq=0，无转矩抖动小
void PhaseOrderSelfTest(void){
    const int16_t test_voltage_mv = 1000;
    const uint16_t step_q15 = 10923; // pi/3 in Q15
    for (int k = 0; k < 100; k++){
        uint16_t angle_u16 = (uint16_t)(k * step_q15);
        setPhaseVoltage(test_voltage_mv, 0, (int16_t)angle_u16);
        HAL_Delay(500);
    }
    setPhaseVoltage(0, 0, 0);
}
