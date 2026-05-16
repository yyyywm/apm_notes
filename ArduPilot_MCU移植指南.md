# ArduPilot MCU 移植指南

## 两种情况的核心区别

```
┌─────────────────────────────────────────────────────────────────────┐
│  情况 A：MCU 已有 ChibiOS 支持                                       │
│  ───────────────────────────                                         │
│  例如：STM32H7 已有 STM32H7xx 目录                                    │
│                                                                     │
│  你不需要修改 modules/ChibiOS/                                       │
│  只需要在 ArduPilot 层做板级配置                                      │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  情况 B：MCU 没有 ChibiOS 支持                                       │
│  ───────────────────────────                                         │
│  例如：全新国产 MCU，ports 下没有对应目录                             │
│                                                                     │
│  你需要先给 ChibiOS 做完整移植，然后才能做 ArduPilot 适配              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 情况 A：MCU 已有 ChibiOS 支持（工作量小）

以 **STM32H7** 为例，ChibiOS 已有 `STM32H7xx` 目录，你需要做的：

### 1. 新增/修改 ArduPilot 板级配置

```
libraries/AP_HAL_ChibiOS/hwdef/你的板子名/
├── hwdef.dat          ← 引脚、时钟、外设分配（主要工作在这里）
├── hwdef_bl.dat       ← Bootloader 配置
└── hwdef.h            ← 生成的头文件（自动生成，不用手写）
```

### 2. hwdef.dat 示例

```python
# 指定 MCU 系列（告诉 ChibiOS 用哪个 port）
MCU STM32H7xx

# 时钟配置
OSCILLATOR_HZ 16000000

# 闪存和 RAM
FLASH_SIZE_KB 2048
RAM_SIZE_KB 512

# SPI 总线
PA5 SPI1_SCK SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# I2C 总线
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1

# UART
PA9 USART1_TX USART1
PA10 USART1_RX USART1

# PWM 输出（电机）
PE9 TIM1_CH1 TIM1 PWM(1)
PE11 TIM1_CH2 TIM1 PWM(2)

# 传感器片选
PA4 BMI270_CS CS
```

### 3. 可能需要微调的文件（如果外设行为有差异）

| 文件 | 场景 |
|------|------|
| `AP_HAL_ChibiOS/SPIDevice.cpp` | SPI 时序/CS 控制特殊 |
| `AP_HAL_ChibiOS/RCOutput.cpp` | 定时器 PWM 特性不同 |
| `AP_HAL_ChibiOS/UARTDriver.cpp` | UART FIFO 深度不同 |

**通常不需要改**，因为 ChibiOS LLD 已经处理了芯片差异。

---

## 情况 B：MCU 没有 ChibiOS 支持（工作量大）

假设你的 MCU 是 **全新的国产芯片**，`modules/ChibiOS/os/hal/ports/` 下没有对应目录。

### 第一步：给 ChibiOS 做移植

```
modules/ChibiOS/
├── os/rt/ports/你的架构/           ← RTOS 内核移植（如果架构全新）
│   ├── chcore.c                   ← 上下文切换、中断入口
│   ├── chcore.h                   ← 寄存器定义、内联汇编
│   └── chsys.c                    ← 系统启动
│
├── os/hal/ports/你的厂商/你的系列/   ← HAL 平台层（类似 STM32F4xx）
│   ├── hal_lld.c                  ← 时钟初始化、低功耗
│   ├── hal_lld.h                  ← 平台宏定义
│   ├── platform.mk                ← 编译规则、选择哪些 LLD
│   └── stm32_registry.h 类似文件   ← 外设存在性/数量定义
│
└── os/hal/ports/你的厂商/LLD/      ← 外设底层驱动（可选复用或新建）
    ├── GPIOv1/ 或 GPIOvX/         ← 如果寄存器架构不同，需新建
    ├── SPIv1/ 或 SPIvX/
    ├── USARTv1/ 或 USARTvX/
    └── ...
```

#### 必须实现的 ChibiOS 文件清单

| 层级 | 文件 | 说明 |
|------|------|------|
| **RTOS 内核** | `chcore.c/h` | 线程上下文切换、中断嵌套、SysTick |
| | `chsys.c` | 系统初始化 |
| **HAL 平台** | `hal_lld.c/h` | 芯片时钟树、复位、低功耗 |
| | `platform.mk` | 选择该芯片用哪些 LLD 驱动 |
| **LLD 外设** | `GPIOvX/` | 引脚输入输出、复用、中断 |
| | `SPIvX/` | SPI 主从、DMA |
| | `USARTvX/` | 串口收发、DMA |
| | `I2CvX/` | I2C 主从、DMA |
| | `TIMvX/` | 定时器、PWM、输入捕获 |
| | `DMAvX/` | DMA 控制器 |
| | `ADCvX/` | ADC 采样 |
| **启动文件** | `startup_xxx.s` | 中断向量表、Reset Handler |
| | `linker_script.ld` | Flash/RAM 地址布局 |

#### CMSIS 头文件（通常由芯片厂商提供）

```c
// 你的厂商应该提供这些：
你的系列.h           ← 外设寄存器结构体、基地址、位定义
system_你的系列.c     ← 系统时钟初始化（SystemInit）
startup_你的系列.s    ← 启动汇编（可由 ChibiOS 提供模板）
```

### 第二步：给 ArduPilot 做 HAL 适配

ChibiOS 移植完成后，参考 `AP_HAL_ChibiOS` 创建：

```
libraries/AP_HAL_你的平台/
├── HAL_你的平台_Class.cpp      ← 实例化所有驱动
├── Scheduler.cpp/.h           ← 任务调度
├── GPIO.cpp/.h                ← GPIO（调用 ChibiOS PAL API）
├── SPIDevice.cpp/.h           ← SPI（调用 ChibiOS SPI API）
├── I2CDevice.cpp/.h           ← I2C
├── UARTDriver.cpp/.h          ← UART
├── RCOutput.cpp/.h            ← PWM
├── RCInput.cpp/.h             ← 遥控输入
├── AnalogIn.cpp/.h            ← ADC
├── Storage.cpp/.h             ← Flash/EEPROM 存储
├── Semaphores.cpp/.h          ← 信号量
├── Util.cpp/.h                ← 工具函数
└── hwdef/                     ← 板级定义
    └── 你的板子名/
        └── hwdef.dat
```

---

## 两种情况的对比总结

| 项目 | 情况 A（已有 ChibiOS） | 情况 B（无 ChibiOS） |
|------|----------------------|---------------------|
| **修改 modules/ChibiOS/** | ❌ 不需要 | ✅ 需要完整移植 |
| **修改 libraries/AP_HAL_ChibiOS/** | ⚠️ 可能微调 | ✅ 需要完整实现 |
| **新增 hwdef.dat** | ✅ 必须 | ✅ 必须 |
| **需要 CMSIS 头文件** | 已有 | 需要厂商提供 |
| **需要启动文件/链接脚本** | 已有 | 需要新建 |
| **工作量** | 几天 | 几个月 |
| **典型例子** | STM32H743 换 H750 | 全新国产 RISC-V MCU |

---

## 关键判断：你的 MCU 属于哪种情况

```bash
# 检查 ChibiOS 是否已有支持
ls modules/ChibiOS/os/hal/ports/

# 如果看到对应厂商/系列目录 → 情况 A
# 如果没有 → 情况 B
```

当前 ChibiOS 支持的架构：

| 厂商 | 系列 |
|------|------|
| STM32 | F0, F1, F3, F37x, F4, F7, G0, G4, H7, L0, L1, L4, L4+, L5, MP1, WB, WL |
| NXP | LPC |
| ADI | ADUCM |
| ST | SPC5 |
| Raspberry Pi | RP (RP2040) |

> **如果你的 MCU 不在列表里，就是情况 B，工作量巨大。** 建议优先考虑已有 ChibiOS 支持的 MCU。
