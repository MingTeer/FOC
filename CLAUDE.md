# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an STM32G070CBT6 microcontroller project for FOC (Field Oriented Control) motor control implementation. The project is currently in v1.1 testing phase with complete peripheral configuration, AS5600 magnetic encoder integration, current sensing, and basic FOC algorithm implementation.

## Build System

**Primary IDE:** Keil µVision MDK-ARM
- Project file: `MDK-ARM/FOC_TESTV1.0.uvprojx`
- Target output: `MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.0.hex` and `.axf`

**Build Commands:**
- **Keil µVision MDK-ARM:** 打开 `MDK-ARM/FOC_TESTV1.0.uvprojx`
  - Build: Project → Build Target (F7)
  - Clean: Project → Clean Targets
  - Flash: Flash → Download (F8)
- **STM32CubeIDE:** File → Open Projects from File System → 选择 `FOC_TESTV1.0.ioc`
- **CLI Flashing:** `STM32_Programmer_CLI -c port=SWD -d MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.0.hex`

**Build Outputs:**
- `MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.0.hex` (用于烧录)
- `MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.0.axf` (调试信息)
- `MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.map` (内存映射)

**Configuration:**
- MCU: STM32G070CBT6 (Cortex-M0+, 128KB Flash, 36KB RAM)
- System Clock: 64MHz (HSI with PLL)
- Debug: ST-Link V2

## Code Architecture

### Project Structure
```
Core/
├── Inc/           # Header files for peripherals and application
├── Src/           # Implementation files
CODE/              # Custom FOC algorithm and sensor libraries
├── AS5600.c/.h    # AS5600 magnetic encoder driver
├── FOC.c/.h       # FOC algorithm implementation
├── fast_math.c/.h # Fast math library with Q1.15 fixed-point
├── pid.c/.h       # PID controller implementation
└── current_sensor.c/.h # Current sensing and processing
MDK-ARM/           # Keil project files and build outputs
Drivers/           # STM32 HAL and CMSIS libraries
```

### Hardware Configuration (STM32CubeMX)
- Configured via `FOC_TESTV1.0.ioc`
- **TIM1:** Advanced control timer for 3-phase PWM generation (PA8, PB3, PB6)
- **TIM3:** Microsecond resolution timer for performance measurement (1MHz clock)
- **ADC1:** Current sensing on Channel 4 (PA4) and Channel 5 (PA5)
- **I2C1:** AS5600 magnetic encoder communication (PB7/PB8, 1MHz Fast Mode Plus)
- **SPI1:** High-speed serial communication (PA1/PA2/PA6)
- **USART1:** Debug output (PA9/PA10)
- **DMA1:** I2C DMA transfers (Channels 1/2)

### Key Components

**AS5600 Magnetic Encoder (CODE/AS5600.c):**
- I2C address: 0x36
- Function: `Get_absolute_angle()` returns rotor angle in degrees (0-360°)
- Resolution: 12-bit (4096 steps per rotation)
- Purpose: Rotor position feedback for FOC

**FOC Algorithm Implementation (CODE/FOC.c, CODE/FOC.h):**
- `FOC_Init()`: Initialize FOC with power supply and voltage limits
- `setPhaseVoltage()`: Set 3-phase voltages using Space Vector PWM
- `electricAngle_position()`: Convert mechanical angle to electrical angle
- `cal_Iq_Id()`: Clarke and Park transformations for current control

**Fast Math Library (CODE/fast_math.c, CODE/fast_math.h):**
- `MCM_Trig_Functions()`: Fast sine/cosine calculation using Q1.15 fixed-point format
- Optimized for real-time FOC calculations without FPU dependency
- Returns `Trig_Components` struct with sin/cos values

**Timing Infrastructure (tim.c):**
- `microsecondTimer_Init()`: Initialize TIM3 for 1µs resolution timing
- `get_microsecond_timestamp()`: Get current timestamp from TIM3 counter
- `get_elapsed_microseconds()`: Calculate elapsed time with proper overflow handling
- **Note:** TIM3 configured as 16-bit counter, handles overflow at 65536µs

**I2C Communication Helpers (i2c.c):**
- `I2C_ScanDevices()`: 扫描I2C总线连接的设备
- `I2C1_ReadRegister_Normal()`: 安全的寄存器读取操作
- `I2C_IsDeviceConnected()`: 设备存在检测

**PID Controller (CODE/pid.c, CODE/pid.h):**
- `PID_Init()`: 初始化PID控制器参数
- `PID_Calculate()`: 位置式PID计算，包含输出限幅
- 支持电流环、速度环和位置环控制
- 结构体包含：kp/ki/kd系数、目标值、积分累积、输出限幅

**Current Sensor (CODE/current_sensor.c, CODE/current_sensor.h):**
- `Current_Sensor_Calibrate_Zero()`: ADC零漂校准，采样1000次取平均值
- `Get_Phase_Currents()`: 获取A、B两相电流值，自动减去零漂偏移
- 支持双相电流采样，使用DMA缓冲区提高效率
- 电压到电流转换，包含参考电压和ADC分辨率参数

### FOC Implementation Status

**Currently Implemented:**
- 硬件外设配置和初始化
- AS5600磁编码器集成和角度读取
- 微秒级定时基础设施用于控制循环
- I2C通信及错误处理
- USART1调试输出
- 基础FOC算法结构和电压控制
- 快速数学库用于三角函数计算
- 空间矢量PWM实现框架
- PID控制器实现（位置式，支持输出限幅）
- 双相电流采样和零漂校准
- Clarke和Park变换电流计算 (cal_Iq_Id)
- 力矩控制和速度合成算法

**Pending FOC Components:**
- 固定频率中断驱动控制循环
- 基于PID的闭环控制实现（电流环、速度环）
- 安全系统（过流、过压保护）
- 空间矢量PWM调制优化
- 电机参数识别和调谐
- 第三相电流计算（使用基尔霍夫定律）

## Development Workflow

1. **Hardware Configuration Changes:**
   - Modify `FOC_TESTV1.0.ioc` in STM32CubeMX
   - Generate code to update peripheral initialization
   - Review changes in `Core/Inc/` and `Core/Src/`

2. **Application Development:**
   - Main application logic in `Core/Src/main.c`
   - Add FOC algorithm implementation between `/* USER CODE BEGIN */` and `/* USER CODE END */` sections
   - Custom libraries in `CODE/` directory

3. **Debugging and Testing:**
   - Use Keil µVision debugger with ST-Link for real-time debugging
   - Serial output via USART1 for telemetry (115200 baud)
   - Use microsecond timing functions for performance measurement

## Important Notes

- **User Code Sections:** Always add custom code within `/* USER CODE BEGIN */` and `/* USER CODE END */` sections to preserve changes during code regeneration
- **Timer Overflow Handling:** Use `get_elapsed_microseconds()` instead of direct subtraction to handle 16-bit timer overflow
- **FOC Algorithm:** Currently not implemented - only sensor testing framework is in place
- **PWM Safety:** TIM1 configured with break functionality for motor safety
- **Current Hardware Configuration:** Only ADC Channel 4 is actively configured for current sensing

## Development Standards

- **Coding Style:** 4-space indentation, UTF-8 encoding, HAL naming conventions
- **Comment Language:** 所有代码注释必须使用中文
- **Communication:** 与用户交流时使用中文回复
- **Commit Format:** Imperative subject lines under 72 characters (e.g., "实现电流控制器")
- **Testing:** Validate on hardware with current-limited supply during development
- **Documentation:** Include telemetry captures and parameter tuning logs in commits

## File Protection

Generated code contains user code protection markers. When modifying:
- Respect `USER CODE BEGIN/END` boundaries
- CubeMX regeneration will preserve code within these sections
- Manual changes outside these sections may be overwritten