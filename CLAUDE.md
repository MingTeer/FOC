# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an STM32G070CBT6 microcontroller project for FOC (Field Oriented Control) motor control implementation. The project is currently in v1.0 testing phase with basic peripheral configuration and AS5600 magnetic encoder integration.

## Build System

**Primary IDE:** Keil µVision MDK-ARM
- Project file: `MDK-ARM/FOC_TESTV1.0.uvprojx`
- Target output: `MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.0.hex` and `.axf`

**Build Commands:**
- Open project in Keil µVision MDK-ARM
- Build: Project → Build Target (F7)
- Clean: Project → Clean Targets
- Flash: Flash → Download (F8)

**Alternative Development:**
- **STM32CubeIDE:** File → Open Projects from File System, point at `FOC_TESTV1.0.ioc`
- **CLI Flashing:** `STM32_Programmer_CLI -c port=SWD -d MDK-ARM/FOC_TESTV1.0/FOC_TESTV1.0.hex`
- **VS Code:** Update `.vscode/launch.json` paths for local environment

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
CODE/              # Custom sensor libraries (AS5600 magnetic encoder)
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
- `I2C_ScanDevices()`: Scan I2C bus for connected devices
- `I2C1_ReadRegister_Normal()`: Safe register read operations
- `I2C_IsDeviceConnected()`: Device presence detection

### FOC Implementation Status

**Currently Implemented:**
- Hardware peripheral configuration and initialization
- AS5600 magnetic encoder integration and angle reading
- Microsecond timing infrastructure for control loops
- I2C communication with error handling
- Debug output via USART1
- Basic FOC algorithm structure with voltage control
- Fast math library for trigonometric calculations
- Space Vector PWM implementation framework

**Pending FOC Components:**
- ADC current sampling and scaling integration
- PI current and speed controllers implementation
- Fixed-frequency interrupt-driven control loop
- Safety systems (overcurrent, overvoltage protection)
- Complete Clarke/Park transformation integration
- Motor parameter identification and tuning

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