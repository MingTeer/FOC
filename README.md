# STM32G070CBT6 FOC电机控制 v1.0

基于STM32G070CBT6微控制器的FOC（磁场定向控制）电机控制项目，集成AS5600磁性编码器支持。

## 项目简介

本项目在STM32G070CBT6微控制器上实现FOC电机控制，提供精确的电机控制功能，配备磁性编码器反馈。目前处于v1.0测试阶段，已完成基础外设配置和传感器集成。

## 硬件配置

### 主要参数
- **微控制器**: STM32G070CBT6 (Cortex-M0+)
- **Flash**: 128KB
- **RAM**: 36KB
- **系统时钟**: 64MHz
- **调试接口**: ST-Link V2

### 引脚定义
| 功能 | 引脚 | 说明 |
|------|------|------|
| PWM_UH | PA8 | U相高侧PWM |
| PWM_VH | PB3 | V相高侧PWM |
| PWM_WH | PB6 | W相高侧PWM |
| ADC_IU | PA4 | U相电流检测 |
| ADC_IV | PA5 | V相电流检测 |
| I2C1_SCL | PB7 | AS5600 I2C时钟 |
| I2C1_SDA | PB8 | AS5600 I2C数据 |
| USART1_TX | PA9 | 调试输出 |

### 传感器
- **AS5600磁性编码器**: I2C���址0x36，12位分辨率(4096步/圈)，角度范围0-360°

## 已实现功能

- ✅ STM32G070CBT6完整外设配置
- ✅ AS5600磁性编码器驱动
- ✅ 微秒级精度定时器
- ✅ I2C通信 (1MHz快速模式+DMA)
- ✅ 三相PWM输出 (TIM1)
- ✅ ADC电流采样
- ✅ USART1调试输出

## 待开发功能

- ⏳ FOC算法完整实现
- ⏳ Clarke/Park变换
- ⏳ PI电流和速度控制器
- ⏳ 空间矢量PWM调制
- ⏳ 固定频率控制循环
- ⏳ 安全保护系统

## 快速开始

### 环境要求
- Keil µVision MDK-ARM (推荐IDE)
- STM32CubeMX (硬件配置)
- ST-Link V2 (调试/烧录)

### 编译和烧录

#### 使用Keil µVision (推荐)
1. 打开 `MDK-ARM/FOC_TESTV1.0.uvprojx`
2. 编译: Project → Build Target (F7)
3. 烧录: Flash → Download (F8)

#### 使用STM32CubeIDE
1. File → Open Projects from File System
2. 选择 `FOC_TESTV1.0.ioc`
3. 在IDE中编译运行

## 项目结构

```
FOC_TESTV1.0/
├── README.md              # 项目说明
├── CLAUDE.md              # Claude Code项目指导
├── FOC_TESTV1.0.ioc       # STM32CubeMX配置文件
├── Core/                  # 应用代码
│   ├── Inc/               # 头文件
│   └── Src/               # 源文件
├── CODE/                  # 自定义传感器库
│   ├── AS5600.c          # AS5600磁性编码器驱动
│   └── AS5600.h          # AS5600头文件
├── Drivers/              # STM32 HAL和CMSIS库
└── MDK-ARM/              # Keil项目文件
    └── FOC_TESTV1.0.uvprojx # Keil项目文件

## 测试和调试

### 传感器测试
当前固件包含完整的传感器测试功能：
- AS5600磁性编码器角度读取验证
- I2C通信鲁棒性测试
- 微秒定时精度验证

### 调试输出
通过USART1监控 (115200波特率)：
- 系统初始化状态
- 传感器读数
- 性能指标
- 错误信息

## 安全注意事项

- **限流保护**: 开发期间务必使用限流电源
- **PWM安全**: TIM1配置了制动功能用于电机安全
- **硬件保护**: 确保适当的隔离和保护电路
- **测试验证**: 完整部署前在硬件上验证所有功能

## 许可证

本项目采用MIT许可证 - 详见 [LICENSE](LICENSE) 文件。