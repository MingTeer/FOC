#include "fast_math.h"
/* 角度 Q1.15：2π -> 65536 刻度 */
#define ANGLE_PI_Q15         32768  /* π   ≈ 180°  */
#define ANGLE_HALF_PI_Q15    16384  /* π/2 ≈  90°  */
#include <stdint.h>

#ifndef ANGLE_PI_Q15
#define ANGLE_PI_Q15         32768
#define ANGLE_HALF_PI_Q15    16384
#endif

/* CORDIC 使用的 atan(2^-i) 角度表，Q1.15 角度格式
 * 标度：2π -> 65536 刻度
 */
static const int16_t hAtan_Cordic_Q15[16] =
{
    8192, 4836, 2555, 1297,  651,  326,  163,   81,
      41,   20,   10,    5,    3,    1,    1,    0
};

// 宏定义部分
#define SIN_COS_TABLE {\
    0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
    0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
    0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
    0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
    0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
    0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
    0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
    0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
    0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
    0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
    0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
    0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
    0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
    0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
    0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
    0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
    0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
    0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
    0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
    0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
    0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
    0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
    0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
    0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
    0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
    0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
    0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
    0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
    0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
    0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
    0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
    0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE}

#define SIN_MASK        0x0300u
#define U0_90           0x0200u
#define U90_180         0x0300u
#define U180_270        0x0000u
#define U270_360        0x0100u

// 查找表变量
const int16_t hSin_Cos_Table[256] = SIN_COS_TABLE;

/**
  * @brief  此函数返回输入角度的余弦和正弦函数
  * @param  hAngle: q1.15格式的角度
  * @retval Trig_Components格式的Sin(angle)和Cos(angle)
  */
Trig_Components MCM_Trig_Functions( int16_t hAngle )
{
  
  int32_t shindex;
  uint16_t uhindex;

  Trig_Components Local_Components;

  /* 10位索引计算 */
  shindex = ( ( int32_t )32768 + ( int32_t )hAngle );
  uhindex = ( uint16_t )shindex;
  uhindex /= ( uint16_t )64;

  switch ( ( uint16_t )( uhindex ) & SIN_MASK )
  {
    case U0_90:
      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( uhindex )];
      Local_Components.hCos = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      break;

    case U90_180:
      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( uhindex )];
      break;

    case U180_270:
      Local_Components.hSin = -hSin_Cos_Table[( uint8_t )( uhindex )];
      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      break;

    case U270_360:
      Local_Components.hSin =  -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      Local_Components.hCos =  hSin_Cos_Table[( uint8_t )( uhindex )];
      break;
    default:
      break;
  }
  return ( Local_Components );
}

/* 在第一象限求 atan(ay/ax)，返回 Q1.15 角度，范围 [0, π/2]
 * 要求：ax >= 0, ay >= 0，不全为 0
 */
static int16_t MCM_Atan_Abs_Q15(uint16_t ax, uint16_t ay)
{
    /* 特殊情况：刚好在坐标轴上 */
    if (ax == 0u)
    {
        return ANGLE_HALF_PI_Q15;   /* 90° */
    }
    if (ay == 0u)
    {
        return 0;                   /* 0°  */
    }

    /* 为了提高精度，把 x,y 先放大 2^8 = 256 倍再做 CORDIC。
     * int32 足够容纳中间结果（安全范围内）。
     */
    int32_t X = ((int32_t)ax) << 8;
    int32_t Y = ((int32_t)ay) << 8;
    int32_t Z = 0;

    for (int i = 0; i < 16; ++i)
    {
        int32_t x_new, y_new;
        if (Y > 0)
        {
            x_new = X + (Y >> i);
            y_new = Y - (X >> i);
            Z    += hAtan_Cordic_Q15[i];
        }
        else
        {
            x_new = X - (Y >> i);
            y_new = Y + (X >> i);
            Z    -= hAtan_Cordic_Q15[i];
        }
        X = x_new;
        Y = y_new;
    }

    return (int16_t)Z;   /* 结果本身范围在 [0, π/2] 之内 */
}

/**
  * @brief  纯整数快速 atan2，和 MCM_Trig_Functions 用同一角度格式
  * @param  y, x : 通常是 Q1.15 或其它定点格式的向量分量
  * @retval Q1.15 角度，范围约为 [-π, π) （即 [-32768, 32767]）
  */
int16_t MCM_Fast_Atan2_Q15(int16_t y, int16_t x)
{
    /* 原点：角度随便定义，这里给 0 */
    if ((x == 0) && (y == 0))
    {
        return 0;
    }

    /* 先到第一象限求绝对值角度 */
    uint16_t ax = (x >= 0) ? (uint16_t)x : (uint16_t)(-x);
    uint16_t ay = (y >= 0) ? (uint16_t)y : (uint16_t)(-y);

    int16_t a = 0;
    if ((ax == 0u) && (ay == 0u))
    {
        a = 0;
    }
    else
    {
        a = MCM_Atan_Abs_Q15(ax, ay);   /* 0 ~ π/2 */
    }

    /* 按象限还原符号（完全仿照标准 atan2 逻辑） */
    int32_t angle;

    if (x >= 0)
    {
        /* 右半平面：第一/第四象限 */
        angle = (y >= 0) ? (int32_t)a : -(int32_t)a;
    }
    else
    {
        /* 左半平面：第二/第三象限 */
        if (y >= 0)
        {
            /* 第二象限：π - a */
            angle = (int32_t)ANGLE_PI_Q15 - (int32_t)a;
        }
        else
        {
            /* 第三象限：a - π  （结果接近 -π） */
            angle = (int32_t)a - (int32_t)ANGLE_PI_Q15;
        }
    }

    /* 压回 int16_t，数值范围自然落在 [-32768,32767] */
    return (int16_t)angle;
}

