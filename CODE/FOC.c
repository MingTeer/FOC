#include "stm32g0xx.h"                  // Device header
#include "math.h"
#include "FOC.h"
#include "tim.h"
#include "AS5600.h"
#include "fast_math.h"
#include "stdio.h"

#define PI 3.1415927f
#define TWO_PI 6.2831853f
#define SQRT3 1.7320508f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define clip(x,min,max) ((x)<(min)?(min):((x)>(max)?(max):(x)))

// 全局变量定义
float power_voltage = 0;        // 电源电压 (V)
float limit_voltage = 0;        // 限制电压 (V)
float zero_electric_angle = 0;  // 零点电角度偏移 (rad)
int pp = 7;                     // 电机极对数

static float inv_power_voltage = 0.0f;  // 电源电压倒数，用于优化计算

/*
 * =================================================================================
 *                            数学处理相关函数
 * =================================================================================
 */

/**
 * @brief  计算电角度
 * @param  shaft_angle  机械角度，单位弧度或度
 * @param  pole_pairs   极对数
 * @retval 电角度，单位与shaft_angle相同
 * @note   电角度 = 机械角度 × 极对数
 */
float electricalAngle(float shaft_angle, int pole_pairs)
{
    return shaft_angle * pole_pairs;
}

/**
 * @brief  弧度归一化到[0, 2PI)
 * @param  angle  输入弧度，单位弧度
 * @retval 归一化后的弧度，范围[0, 2PI)
 * @note   使用取余与偏移保证角度在指定范围内
 */
float normallizeRadian(float angle)
{
    float temp = fmodf(angle, 2.0f * PI);
    if (temp < 0.0f)
    {
        temp += 2.0f * PI;
    }
    return temp;
}

/**
 * @brief  角度归一化到[-180°, 180°)
 * @param  angle  输入角度，单位度
 * @retval 归一化后的角度，范围[-180°, 180°)
 * @note   便于角度正负计算和显示
 */
float normallizeDegree(float angle)
{
    // 将角度规范到[-180, 180)范围
    float temp = fmodf(angle + 180.0f, 360.0f);
    if (temp < 0.0f)
    {
        temp += 360.0f;
    }
    return temp - 180.0f;
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
float electricAngle_position(void)
{
    return normallizeRadian((Get_absolute_angle() / 360.0f * 2 * PI * pp) - zero_electric_angle);
}

/**
 * @brief  传感器校准流程
 * @param  void
 * @retval void
 * @note   通过外加转矩使电机到达已知位置，确保电角度零点校准
 */
void Check_Sensor(void)
{
    const float align_voltage = limit_voltage * 0.5f;
    const uint32_t settle_delay_ms = 800;
    const uint16_t sample_count = 64;

    setPhaseVoltage(0.0f, align_voltage, 0.0f);
    HAL_Delay(settle_delay_ms);

    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    float last_angle = 0.0f;
    for (uint16_t i = 0; i < sample_count; i++)
    {
        float mechanical_angle_deg = Get_absolute_angle();
        float electrical_angle = mechanical_angle_deg / 360.0f * TWO_PI * pp;
        sin_sum += sinf(electrical_angle);
        cos_sum += cosf(electrical_angle);
        last_angle = electrical_angle;
        HAL_Delay(5);
    }

    float magnitude = sin_sum * sin_sum + cos_sum * cos_sum;
    float average_angle = (magnitude > 1e-6f) ? atan2f(sin_sum, cos_sum) : last_angle;
    zero_electric_angle = normallizeRadian(average_angle);

    setPhaseVoltage(0.0f, 0.0f, 0.0f);
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
void set_power_voltage(float ua,float ub,float uc)
{
    // 电压限制在允许范围内
    ua = clip(ua,0.0f, power_voltage);
    ub = clip(ub, 0.0f, power_voltage);
    uc = clip(uc, 0.0f, power_voltage);

    // 转换为PWM占空比 (0-1)
    float duty_a = clip(ua * inv_power_voltage, 0.0f, 1.0f);
    float duty_b = clip(ub * inv_power_voltage, 0.0f, 1.0f);
    float duty_c = clip(uc * inv_power_voltage, 0.0f, 1.0f);

    // 设置PWM占空比 (例如*1000以适应定时器范围)
    TIM1_CH1_SetDutyCycle(duty_a * 1000);
    TIM1_CH2_SetDutyCycle(duty_b * 1000);
    TIM1_CH3_SetDutyCycle(duty_c * 1000);
}

/**
 * @brief  设置空间矢量PWM输出 (FOC核心算法)
 * @param  Uq            q轴电压分量，单位V
 * @param  Ud            d轴电压分量，单位V
 * @param  angle_eletric 电角度，单位弧度
 * @note   实现Park变换和空间矢量PWM算法
 */
void setPhaseVoltage(float Uq,float Ud,float angle_eletric)
{
    // 1. 角度归一化: 将任意角度转换为[-PI, PI)范围
    float angle_norm = angle_eletric;
    if (angle_norm >= PI)
    {
        angle_norm -= TWO_PI;
    }
    else if (angle_norm < -PI)
    {
        angle_norm += TWO_PI;
    }
    int16_t angle_q15 = (int16_t)(angle_norm * 32768.0f / PI);

    // 2. 使用查表法快速计算sin与cos
    Trig_Components components = MCM_Trig_Functions(angle_q15);

    // 3. 将Q1.15格式转换回浮点数
    float sin_val = (float)components.hSin / 32768.0f;
    float cos_val = (float)components.hCos / 32768.0f;

    // 4. Park变换 (此处仅使用Uq分量)
    float Uout = sqrtf(Ud * Ud + Uq * Uq);
    if (Uout > limit_voltage)
    {
        float scale = limit_voltage / Uout;
        Ud *= scale;
        Uq *= scale;
    }

    float Ualpha = Ud * cos_val - Uq * sin_val;
    float Ubeta = Ud * sin_val + Uq * cos_val;


    // 5. Clarke变换: αβ -> 三相电压
    float Vdc_half = power_voltage * 0.5f;  // 直流母线电压一半
    float Ua = Ualpha + Vdc_half;           // A相电压
    float Ub = (-0.5f * Ualpha + SQRT3 * 0.5f * Ubeta) + Vdc_half;  // B相电压
    float Uc = (-0.5f * Ualpha - SQRT3 * 0.5f * Ubeta) + Vdc_half;  // C相电压

    // 6. 设置三相电压
    set_power_voltage(Ua, Ub, Uc);
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
void cal_Iq_Id(float current_a, float current_b, float angle_el, float *iq_out, float *id_out)
{
    float I_alpha = current_a;
    float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

    // 使用查表法求 sin, cos
    float angle_norm = angle_el;
    if (angle_norm >= PI)
    {
        angle_norm -= TWO_PI;
    }
    else if (angle_norm < -PI)
    {
        angle_norm += TWO_PI;
    }
    int16_t angle_q15 = (int16_t)(angle_norm * 32768.0f / PI);

    Trig_Components components = MCM_Trig_Functions(angle_q15);
    float st = (float)components.hSin / 32768.0f;
    float ct = (float)components.hCos / 32768.0f;

    // Park变换：计算dq轴电流
    float I_q = I_beta * ct - I_alpha * st;
    float I_d = I_alpha * ct + I_beta * st;

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
 * @param  l_voltage  限制电压，单位V
 * @retval void
 * @note   初始化电压参数，执行传感器校准，设置安全限制
 */
void FOC_Init(float p_voltage,float l_voltage)
{
    // 设置电源电压参数
    power_voltage = p_voltage;
    if (power_voltage > 0.001f)
    {
        inv_power_voltage = 1.0f / power_voltage;  // 提前计算倒数优化性能
    }
    else
    {
        inv_power_voltage = 0.0f;
    }

    // 初始化限制电压
    limit_voltage = p_voltage / 2;

    // 执行传感器校准
    Check_Sensor();

    // 设置用户定义的电压限制
    limit_voltage = l_voltage;
}

// 仅做自检：Ud>0 使磁场对齐 d 轴、Uq=0，无转矩抖动小
void PhaseOrderSelfTest(void){
    for (int k = 0; k < 100; k++){
        setPhaseVoltage(1.0f, 0.0f, k * (PI/3));
        HAL_Delay(500);
    }
    setPhaseVoltage(0,0,0);
}


