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
- ✅ 快速数学库（Q1.15定点三角函数）
- ✅ PID控制器（位置式，支持输出限幅）
- ✅ FOC算法基础框架

## 待开发功能

- ⏳ 完整的Clarke/Park变换集成
- ⏳ 基于PID的电流环和速度环控制
- ⏳ 固定频率中断驱动控制循环
- ⏳ 安全保护系统（过流、过压保护）
- ⏳ 空间矢量PWM调制优化
- ⏳ 电机参数识别和调谐

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
│   ├── AS5600.c/.h        # AS5600磁性编码器驱动
│   ├── FOC.c/.h           # FOC算法实现
│   ├── fast_math.c/.h     # 快速数学库
│   ├── pid.c/.h           # PID控制器
│   └── current_sensor.c/.h # 电流传感器
├── Drivers/               # STM32 HAL和CMSIS库
└── MDK-ARM/               # Keil项目文件
    ├── FOC_TESTV1.0.uvprojx # Keil项目文件
    ├── FOC_TESTV1.0.hex   # 烧录文件
    └── FOC_TESTV1.0.axf   # 调试文件
```

## 测试和调试

### 传感器测试
当前固件包含完整的传感器测试功能：
- AS5600磁性编码器角度读取验证
- I2C通信鲁棒性测试
- 微秒定时精度验证
- ADC电流采样功能测试

### 核心算法组件
- **FOC算法结构**: `electricalAngle()`, `setPhaseVoltage()`, `cal_Iq_Id()`
- **快速数学库**: Q1.15定点格式sine/cosine计算，无FPU依赖
- **PID控制器**: 位置式PID，支持输出限幅，适用于电流环、速度环和位置环
- **定时系统**: TIM3提供1µs分辨率，支持16位溢出处理

### 调试输出
通过USART1监控 (115200波特率)：
- 系统初始化状态
- 传感器读数
- 性能指标
- 错误信息

## 开发指南

### 代码规范
- **编码风格**: 4空格缩进，UTF-8编码，HAL命名约定
- **注释语言**: 所有代码注释必须使用中文
- **用户代码区**: 在 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间添加自定义代码
- **定时器溢出**: 使用 `get_elapsed_microseconds()` 而非直接减法处理16位定时器溢出

### 提交规范
- **格式**: 祈使句主题行，72字符以内（如："实现PID控制器"）
- **测试**: 在硬件上验证，开发期间使用限流电源
- **文档**: 在提交中包含遥测数据和参数调谐日志

## 安全注意事项

- **限流保护**: 开发期间务必使用限流电源
- **PWM安全**: TIM1配置了制动功能用于电机安全
- **硬件保护**: 确保适当的隔离和保护电路
- **测试验证**: 完整部署前在硬件上验证所有功能

## 许可证

本项目采用MIT许可证 - 详见 [LICENSE](LICENSE) 文件。