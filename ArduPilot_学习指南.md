# ArduPilot 飞控项目完整学习指南

> 本文档旨在帮助飞控开发初学者系统学习ArduPilot开源飞控项目，掌握飞控设计的框架、架构、模块功能及控制算法。

## 目录

1. [项目概述](#1-项目概述)
2. [整体架构](#2-整体架构)
3. [核心框架详解](#3-核心框架详解)
4. [传感器系统](#4-传感器系统)
5. [姿态估计与导航](#5-姿态估计与导航)
6. [控制系统](#6-控制系统)
7. [参数系统](#7-参数系统)
8. [通信系统](#8-通信系统)
9. [飞行器类型](#9-飞行器类型)
10. [学习路径与实践建议](#10-学习路径与实践建议)

---

## 1. 项目概述

### 1.1 什么是ArduPilot

ArduPilot是世界上最成熟的开源无人机飞控项目之一，由全球开发者社区维护，经过十余年的发展和完善。其代码质量高、架构清晰、文档完善，是学习飞控开发的最佳选择。

### 1.2 支持的飞行器类型

| 类型 | 目录 | 说明 |
|------|------|------|
| 多旋翼 | `ArduCopter/` | 四旋翼、六旋翼、八旋翼等 |
| 固定翼 | `ArduPlane/` | 常规飞机、VTOL倾转旋翼等 |
| 无人车 | `Rover/` | 无人驾驶车辆 |
| 水下机器人 | `ArduSub/` | 无人潜水器 |
| 天线追踪器 | `AntennaTracker/` | 追踪移动目标 |
| 浮空器 | `Blimp/` | 气球或飞艇 |

### 1.3 代码仓库结构

```
ardupilot/
├── ArduCopter/        # 多旋翼飞控代码（入门推荐从此开始）
├── ArduPlane/        # 固定翼飞控代码
├── ArduSub/          # 水下机器人代码
├── Rover/            # 无人车代码
├── AntennaTracker/   # 天线追踪器代码
├── Blimp/            # 浮空器代码
├── libraries/        # 核心库（最重要！理解架构的关键）
├── Tools/            # 工具集（SITL仿真、编译工具等）
├── modules/          # 外部依赖模块
├── docs/             # 官方文档
├── tests/            # 测试代码
└── waf / wscript    # 构建系统配置
```

### 1.4 核心库目录详解

```
libraries/
├── AP_HAL/              # 【硬件抽象层】隔离硬件差异
├── AP_Common/           # 公共基础定义和工具
├── AP_Param/            # 【参数管理系统】配置的核心
├── AP_Scheduler/        # 【任务调度器】飞控的心跳
├── AP_Vehicle/          # 【飞控基类】所有飞行器的父类
├── AP_AHRS/             # 【姿态航向参考系统】姿态估计
├── AP_InertialSensor/   # IMU传感器驱动
├── AP_GPS/             # GPS接收机驱动
├── AP_Baro/            # 气压计驱动
├── AP_Compass/         # 磁力计驱动
├── AP_NavEKF3/         # 扩展卡尔曼滤波（位置/速度估计）
├── AC_AttitudeControl/ # 姿态控制器
├── AC_WPNav/           # 航点导航
├── GCS_MAVLink/        # MAVLink地面站通信
├── Filter/             # 数字滤波器
├── AP_Math/           # 数学库（向量、矩阵运算）
├── PID/                # PID控制器
└── ...                 # 更多专用库
```

---

## 2. 整体架构

### 2.1 分层架构设计

ArduPilot采用经典的**分层架构**设计，这是企业级软件工程的最佳实践：

```
┌─────────────────────────────────────────────────────────────────┐
│                      应用层 (Application Layer)                   │
│                                                                  │
│          ArduCopter    │    ArduPlane    │    Rover            │
│          (多旋翼)       │    (固定翼)      │    (无人车)          │
├─────────────────────────────────────────────────────────────────┤
│                      框架层 (Framework Layer)                    │
│                                                                  │
│     AP_Vehicle      │  AP_Scheduler  │  AP_Param  │  AP_AHRS  │
│     (飞控基类)       │  (任务调度)     │  (参数管理) │  (姿态)    │
├─────────────────────────────────────────────────────────────────┤
│                   硬件抽象层 (HAL - Hardware Abstraction Layer)    │
│                                                                  │
│     AP_HAL_ChibiOS   │   AP_HAL_SITL   │   AP_HAL_Linux        │
│     (STM32嵌入式)    │   (软件仿真)     │   (Linux平台)          │
├─────────────────────────────────────────────────────────────────┤
│                        硬件层 (Hardware Layer)                    │
│                                                                  │
│       STM32F7       │    Linux/SITL    │    ESP32              │
│       (飞控板)      │    (电脑仿真)     │    (ESP32飞控)         │
└─────────────────────────────────────────────────────────────────┘

分层设计的好处：
1. 硬件无关：同一套代码可以在不同硬件上运行
2. 模块化：每个模块职责清晰，易于维护
3. 可扩展：添加新硬件只需实现HAL接口
4. 可测试：可以在SITL仿真中测试所有功能
```

### 2.2 核心类继承关系

```
┌─────────────────────────────────────────────────────────────────┐
│                   AP_HAL::HAL (硬件抽象层接口)                    │
│                   提供: 串口、GPIO、定时器等硬件访问               │
└─────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                      AP_Vehicle (飞控基类)                        │
│                                                                   │
│   包含通用子系统:                                                 │
│   ├── AP_Scheduler      - 任务调度器                             │
│   ├── AP_InertialSensor - IMU传感器                              │
│   ├── AP_GPS            - GPS接收机                              │
│   ├── AP_Baro           - 气压计                                 │
│   ├── AP_Compass        - 磁力计                                 │
│   ├── AP_AHRS           - 姿态估计                               │
│   ├── AP_Logger         - 日志记录                               │
│   ├── AP_SerialManager  - 串口管理                               │
│   └── ...              - 更多                                   │
└─────────────────────────────────────────────────────────────────┘
                               │
           ┌───────────────────┼───────────────────┐
           ▼                   ▼                   ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   ArduCopter    │  │   ArduPlane     │  │     Rover       │
│   (多旋翼飞控)   │  │   (固定翼飞控)   │  │    (无人车)     │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

### 2.3 单例模式 (Singleton Pattern)

ArduPilot广泛使用**单例模式**来管理全局对象，确保整个系统只有一个实例：

```cpp
// 全局单例访问宏（定义在各个库中）
AP::scheduler()      // 获取任务调度器实例
AP::ahrs()          // 获取姿态估计器实例
AP::gps()           // 获取GPS实例
AP::ins()           // 获取IMU传感器实例
AP::compass()       // 获取磁力计实例
AP::barometer()     // 获取气压计实例
AP::param()         // 获取参数系统实例
AP::vehicle()       // 获取飞控实例
AP::logger()        // 获取日志实例

// 使用示例
void my_function() {
    // 获取当前姿态
    float roll = AP::ahrs().get_roll_deg();
    float pitch = AP::ahrs().get_pitch_deg();

    // 获取GPS数据
    if (AP::gps()->status() >= GPS_FIX_3D) {
        float lat = AP::gps()->location().lat;
    }

    // 获取IMU数据
    Vector3f gyro = AP::ins().get_gyro();
}
```

**为什么要用单例？**
- 整个系统只需要一个实例（传感器、调度器等）
- 方便全局访问，避免复杂的依赖注入
- 减少内存占用，简化代码

---

## 3. 核心框架详解

### 3.1 AP_Vehicle - 飞控基类

**文件位置**: `libraries/AP_Vehicle/AP_Vehicle.h`

AP_Vehicle是所有飞行器的**基类**，定义了通用接口和公共子系统。

#### 3.1.1 核心成员变量详解

```cpp
class AP_Vehicle : public AP_HAL::HAL::Callbacks {
protected:
    // ========== 【任务调度器】 ==========
    // 飞控的心跳系统，决定每个任务何时执行
    AP_Scheduler scheduler;

    // ========== 【传感器系统】 ==========
    // GPS接收机 - 提供位置、速度、时间
    AP_GPS gps;

    // 气压计 - 提供高度估计
    AP_Baro barometer;

    // 磁力计 - 提供航向角
    Compass compass;

    // IMU传感器 - 提供加速度和角速度
    AP_InertialSensor ins;

    // 按钮 - 物理按钮输入
    AP_Button button;

    // 距离传感器 - 测距（定高、避障）
    RangeFinder rangefinder;

    // ========== 【状态估计】 ==========
    // 姿态航向参考系统 - 融合传感器数据，估计飞行器状态
    AP_AHRS ahrs;

    // ========== 【通信】 ==========
    // 串口管理器 - 管理多个串口
    AP_SerialManager serial_manager;

    // CAN总线管理器 - 连接CAN设备（电调、传感器等）
    AP_CANManager can_mgr;

    // ========== 【通知】 ==========
    // LED、蜂鸣器等状态指示
    AP_Notify notify;

    // ========== 【日志】 ==========
    // 飞行数据记录
    AP_Logger logger;

    // ========== 【时间管理】 ==========
    // 积分时间 - 上一帧循环耗时，用于积分计算
    float G_Dt;
};
```

#### 3.1.2 核心方法详解

```cpp
class AP_Vehicle : public AP_HAL::HAL::Callbacks {
public:
    /**
     * 初始化入口 - HAL层调用
     * 注意：使用final关键字防止子类覆盖，强制使用init_ardupilot()
     */
    void setup(void) override final;

    /**
     * 主循环 - HAL层调用
     * 内部调用scheduler.loop()执行任务调度
     */
    void loop() override final;

    // ========== 虚方法（由子类实现） ==========

    /**
     * 子类特定的初始化
     * ArduCopter实现: 初始化电机、传感器、飞行模式等
     */
    virtual void init_ardupilot() = 0;

    /**
     * 加载参数
     * 从EEPROM加载保存的配置
     */
    virtual void load_parameters() = 0;

    /**
     * 设置飞行模式
     * @param new_mode 新的模式编号
     * @param reason 模式切换原因
     * @return 是否切换成功
     */
    virtual bool set_mode(const uint8_t new_mode, const ModeReason reason) = 0;

    /**
     * 获取当前飞行模式
     */
    virtual uint8_t get_mode() const = 0;
};
```

**设计意图说明：**
- `setup()` 和 `loop()` 是HAL层调用的入口
- `final` 关键字强制子类使用 `init_ardupilot()` 和 `load_parameters()` 进行定制化初始化
- 所有传感器、子系统都在AP_Vehicle中声明，子类直接继承使用

### 3.2 AP_Scheduler - 任务调度器

**文件位置**: `libraries/AP_Scheduler/AP_Scheduler.h`

任务调度器是飞控的**"心跳"**，决定每个任务在何时执行。这是实时系统的核心组件。

#### 3.2.1 任务定义结构

```cpp
struct Task {
    task_fn_t function;          // 任务函数指针（void func(void)类型）
    const char *name;            // 任务名称（用于日志和调试）
    float rate_hz;               // 运行频率 (Hz)，0表示最快频率
    uint16_t max_time_micros;   // 最大允许执行时间（微秒），用于性能监控
    uint8_t priority;            // 优先级，数字越大优先级越高
};

/**
 * 任务定义宏 - 用于简化任务定义
 *
 * @param classname  所属的类名
 * @param classptr   类的实例指针
 * @param func       要调用的方法名
 * @param _rate_hz   执行频率（Hz）
 * @param _max_time_micros 最大执行时间（微秒）
 * @param _priority  优先级
 *
 * 使用示例：
 * SCHED_TASK_CLASS(Copter, &copter, update_GPS, 50, 100, 0)
 * 表示：copter.update_GPS() 每20ms调用一次，最大执行100微秒
 */
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros, _priority) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(classname, func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros,\
    .priority = _priority \
}
```

#### 3.2.2 多旋翼任务列表示例

```cpp
// 文件: ArduCopter/Copter.cpp

// 任务定义表 - 定义所有需要在循环中执行的任务
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // 【快速任务】最高优先级，每帧执行
    { &Copter::fast_loop,           "fast_loop",           0,    0,   0 },

    // 【GPS任务】50Hz - 每20ms执行一次
    { &Copter::update_GPS,          "GPS",                50,  100,   0 },

    // 【AHRS任务】50Hz - 更新姿态估计
    { &Copter::update_ahrs,         "AHRS",               50,  100,   0 },

    // 【悬停油门】50Hz - 计算悬停油门
    { &Copter::update_throttle_hover, "ThrottleHover",     50,  100,   0 },

    // 【电池罗盘】10Hz - 每100ms执行
    { &Copter::update_batt_compass, "BattCompass",        10,  100,   0 },

    // 【1Hz任务】每秒执行一次
    { &Copter::one_hz_loop,         "OneHz",               1,  100,   0 },

    // 【飞控模式更新】50Hz
    { &Copter::update_flight_mode,  "FlightMode",         50,  100,   0 },

    // 【日志记录】50Hz
    { &Copter::ten_hz_logging_loop, "Log",                50,  200,   0 },
};

/**
 * 频率计算规则：
 * - 基础循环频率 = 400Hz（多旋翼默认）
 * - rate_hz = 50 意味着每 400/50 = 8 个tick执行一次
 * - rate_hz = 10 意味着每 400/10 = 40 个tick执行一次
 * - rate_hz = 1  意味着每 400/1 = 400 个tick执行一次
 */
```

#### 3.2.3 fast_loop - 核心循环

```cpp
/**
 * fast_loop() - 最高频率任务（400Hz）
 * 这是飞控最关键的任务，包含姿态控制和安全检查
 */
void Copter::fast_loop() {
    // ========== 1. 读取传感器 ==========
    // 读取IMU数据（加速度计、陀螺仪）
    read_inertia();

    // 读取GPS数据
    update_GPS();

    // 读取气压计
    read_barometer();

    // 读取距离传感器
    read_rangefinder();

    // ========== 2. 更新状态估计 ==========
    // AHRS使用IMU数据更新姿态估计
    update_AHRS();

    // ========== 3. 更新飞行模式 ==========
    // 根据当前模式处理输入和输出
    update_flight_mode();

    // ========== 4. 执行控制 ==========
    // 角速度控制器 - 计算电机输出
    attitude_control->rate_controller_run();

    // ========== 5. 输出到电机 ==========
    motors_output();
}

/**
 * one_hz_loop() - 低频率任务（1Hz）
 * 执行不需要高频率更新的任务
 */
void Copter::one_hz_loop() {
    // 电池状态检查
    battery.check();

    // GPS状态检查
    gps.check();

    // 更新LED状态
    notify.update();

    // 保存日志
    Log_Write_Data(LogDataID::AP_STATE, 0);
}
```

#### 3.2.4 调度器核心方法

```cpp
class AP_Scheduler {
public:
    /**
     * 初始化调度器
     * @param tasks      任务数组
     * @param num_tasks  任务数量
     * @param log_performance_bit 性能日志位掩码
     */
    void init(const Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit);

    /**
     * 主循环调用 - 执行任务调度
     * 这是被HAL.loop()调用的核心函数
     */
    void loop();

    /**
     * 时钟tick - 更新时钟计数器
     * 每次循环调用一次
     */
    void tick(void);

    /**
     * 运行任务
     * @param time_available 本次循环可用时间（微秒）
     */
    void run(uint32_t time_available);

    /**
     * 获取当前循环频率（Hz）
     * 多旋翼默认400Hz
     */
    uint16_t get_loop_rate_hz(void) {
        return _active_loop_rate_hz;
    }

    /**
     * 获取循环负载（0.0-1.0之间）
     * 1.0表示100%负载，CPU满负荷运行
     */
    float load_average();

    /**
     * 获取最后循环耗时（秒）
     */
    float get_last_loop_time_s(void) const {
        return _last_loop_time_s;
    }
};
```

#### 3.2.5 调度原理图解

```
主循环流程：
┌─────────────────────────────────────────────────────────────────┐
│                     HAL.main_loop() 被定时器调用                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │ 1. scheduler.tick() - 更新时钟                              │  │
│  │    _tick_counter++                                         │  │
│  │    _loop_sample_time_us = 当前时间                          │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │ 2. 计算可用时间                                             │  │
│  │    loop_period = 1秒 / 400Hz = 2500微秒                    │  │
│  │    available = loop_period - 上一帧耗时                     │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │ 3. scheduler.run(available) - 执行到期的任务                │  │
│  │                                                             │  │
│  │    For 每个任务:                                            │  │
│  │      if (任务应该执行) {                                    │  │
│  │         记录开始时间                                        │  │
│  │         执行任务函数                                        │  │
│  │         记录结束时间                                        │  │
│  │         如果超时，标记警告                                  │  │
│  │      }                                                      │  │
│  │                                                             │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │ 4. 计算负载并记录性能                                       │  │
│  │    _last_loop_time_s = 实际耗时                             │  │
│  │    load = 1.0 - spare_time / loop_period                   │  │
│  └─────────────────────────────────────────────────────────────┘  │
│                                                                     │
└─────────────────────────────────────────────────────────────────┘

任务执行判断示例：
- 基础频率 = 400Hz
- 任务A: rate_hz = 400 → 每1个tick执行一次 → 每2.5ms
- 任务B: rate_hz = 200 → 每2个tick执行一次 → 每5ms
- 任务C: rate_hz = 50  → 每8个tick执行一次 → 每20ms
- 任务D: rate_hz = 10  → 每40个tick执行一次 → 每100ms
```

**为什么要这样设计？**
1. **确定性**：高优先级任务（姿态控制）总是优先执行
2. **负载均衡**：自动根据CPU负载调整任务执行
3. **可监控**：记录每个任务的执行时间，便于调试
4. **可配置**：可以通过参数调整循环频率

### 3.3 HAL (Hardware Abstraction Layer) - 硬件抽象层

**文件位置**: `libraries/AP_HAL/`

HAL层将硬件操作抽象成**统一接口**，使上层代码与具体硬件解耦。

#### 3.3.1 HAL类结构

```cpp
// 文件: libraries/AP_HAL/HAL.h

class HAL {
public:
    /**
     * 获取串口驱动
     * @param num 串口号 (0, 1, 2, ...)
     * @return 串口驱动指针
     */
    UARTDriver *serial(int num);

    // ========== 通用硬件接口 ==========
    GPIO *gpio;                    // GPIO引脚控制
    RTC *rtc;                     // 实时时钟
    System *system;               // 系统控制
    Storage *storage;             // 持久化存储

    // ========== 模拟/数字 ==========
    AnalogIn *analogin;           // 模拟输入（电池电压检测）

    // ========== RC输入输出 ==========
    RCOutput *rcout;              // PWM输出（电机、舵机）
    RCInput *rcin;               // PWM输入（接收机）

    // ========== 调度 ==========
    Scheduler *scheduler;         // 任务调度器

    // ========== 示波器（调试） ==========
    void (*console_backend)(AP_HAL::BetterStream *);
};
```

#### 3.3.2 HAL实现类

| 实现类 | 平台 | 用途 |
|--------|------|------|
| `AP_HAL_ChibiOS::HAL` | STM32 | 主要嵌入式飞控平台 |
| `AP_HAL_SITL::HAL` | Linux/Windows | 软件在环仿真 |
| `AP_HAL_Linux::HAL` | Linux | Linux原生运行 |
| `AP_HAL_ESP32::HAL` | ESP32 | ESP32飞控 |

#### 3.3.3 全局HAL访问

```cpp
// 全局HAL对象声明（由各平台HAL实现提供）
extern const AP_HAL::HAL& hal;

// ========== 常用操作示例 ==========

// 1. 串口输出
void uart_example() {
    AP_HAL::UARTDriver *uart = hal.serial(0);  // 串口0通常是USB/调试串口
    uart->printf("Hello, World!\n");
    uart->printf("Value: %d, Float: %.2f\n", 123, 45.67);
}

// 2. GPIO控制
void gpio_example() {
    // 设置引脚模式
    hal.gpio->pinMode(13, OUTPUT);  // LED引脚

    // 写引脚
    hal.gpio->digitalWrite(13, HIGH);  // LED亮
    hal.gpio->digitalWrite(13, LOW);   // LED灭

    // 读引脚
    bool value = hal.gpio->digitalRead(12);
}

// 3. PWM输出（电机控制）
void pwm_example() {
    // 启用通道
    hal.rcout->enable_ch(0);  // 通道0
    hal.rcout->enable_ch(1);  // 通道1

    // 设置PWM值 (1000-2000μs)
    hal.rcout->write(0, 1500);  // 通道0输出1500μs
    hal.rcout->write(1, 1000);  // 通道1输出1000μs
}

// 4. PWM输入（接收机）
void pwm_input_example() {
    uint16_t channels[8];
    hal.rcin->read(channels, 8);  // 读取8个通道
    uint16_t throttle = channels[2];  // 通常通道2是油门
}

// 5. 延时
void delay_example() {
    hal.scheduler->delay(100);              // 阻塞延时100ms
    hal.scheduler->delay_microseconds(100);  // 阻塞延时100μs
}
```

#### 3.3.4 时间函数

```cpp
// ========== 时间获取 ==========

// 毫秒计数器
uint32_t ms = AP_HAL::millis();       // 32位毫秒（约49天溢出）
uint64_t ms64 = AP_HAL::millis64();   // 64位毫秒（永不过期）

// 微秒计数器
uint32_t us = AP_HAL::micros();       // 32位微秒（约71分钟溢出）
uint64_t us64 = AP_HAL::micros64();   // 64位微秒（永不过期）

// ========== 时间使用示例 ==========

// 1. 计算时间差
uint32_t start_time = AP_HAL::millis();
do_something();
uint32_t elapsed = AP_HAL::millis() - start_time;

// 2. 限制定时执行
static uint32_t last_print_ms = 0;
if (AP_HAL::millis() - last_print_ms > 1000) {  // 每秒执行一次
    last_print_ms = AP_HAL::millis();
    // 执行操作
}

// 3. 非阻塞延时（在循环中使用）
#define PRINT_INTERVAL 1000  // 1000ms
uint32_t last_print = 0;

void loop() {
    uint32_t now = AP_HAL::millis();
    if (now - last_print >= PRINT_INTERVAL) {
        last_print = now;
        // 每秒执行一次的操作
    }
}
```

**HAL设计的好处：**
1. **硬件无关**：同一套代码可以运行在STM32、ESP32、Linux等不同平台
2. **易于移植**：新硬件支持只需实现HAL接口
3. **便于测试**：可以在SITL仿真中测试所有功能逻辑

---

## 4. 传感器系统

### 4.1 AP_InertialSensor - IMU传感器

**文件位置**: `libraries/AP_InertialSensor/`

IMU（Inertial Measurement Unit，惯性测量单元）是飞控的"感觉器官"，包含**加速度计**和**陀螺仪**。

#### 4.1.1 传感器坐标系

```
传感器坐标系（右手定则）：
        Z (向上)
         ↑
         │
         │→ Y (向右)
        ╱
       ╱
      ↙ X (向前)

对于安装在无人机上的IMU：
- X轴：指向机头方向
- Y轴：指向机体右侧
- Z轴：指向机体下方（通常朝下）

注意：坐标系方向取决于传感器安装方向
```

#### 4.1.2 IMU类结构

```cpp
class AP_InertialSensor {
public:
    // ========== 数据获取方法 ==========

    /**
     * 获取陀螺仪数据（原始/未滤波）
     * @param instance IMU实例编号 (0, 1, 2...)
     * @return 角速度向量，单位：弧度/秒
     *
     * gyro.x = 绕X轴旋转速度（Roll rate）
     * gyro.y = 绕Y轴旋转速度（Pitch rate）
     * gyro.z = 绕Z轴旋转速度（Yaw rate）
     */
    const Vector3f& get_gyro(uint8_t instance=0) const;

    /**
     * 获取加速度计数据
     * @param instance IMU实例编号
     * @return 加速度向量，单位：m/s²
     *
     * accel.z ≈ 9.8 表示水平放置
     * accel.x/y ≈ 0 表示水平放置
     */
    const Vector3f& get_accel(uint8_t instance=0) const;

    /**
     * 检测是否新的陀螺仪偏移数据
     */
    bool get_new_gyro_offsets(Vector3f &offsets) const;

    /**
     * 获取传感器温度（用于温度补偿）
     */
    float get_temperature(uint8_t instance=0) const;

    /**
     * 检查IMU是否健康
     */
    bool healthy(uint8_t instance=0) const;

    /**
     * 获取样本计数（用于同步）
     */
    uint32_t get_sample_count() const { return _sample_count; }

    /**
     * 获取样本时间戳（微秒）
     */
    uint64_t get_sample_time_us(uint8_t instance=0) const;
};
```

#### 4.1.3 IMU使用示例

```cpp
// ========== 1. 获取IMU数据 ==========

void read_imu() {
    // 获取主IMU（实例0）
    Vector3f gyro = AP::ins().get_gyro();
    Vector3f accel = AP::ins().get_accel();

    // 打印数据（调试用）
    hal.console->printf("Gyro: %.2f, %.2f, %.2f rad/s\n",
                        degrees(gyro.x), degrees(gyro.y), degrees(gyro.z));
    hal.console->printf("Accel: %.2f, %.2f, %.2f m/s²\n",
                        accel.x, accel.y, accel.z);

    // 检查健康状态
    if (!AP::ins().healthy()) {
        // IMU不健康，需要处理
    }
}

// ========== 2. 多IMU冗余 ==========

void multi_imu_example() {
    uint8_t num_imus = AP::ins().get_num_instances();

    for (uint8_t i = 0; i < num_imus; i++) {
        Vector3f gyro = AP::ins().get_gyro(i);
        Vector3f accel = AP::ins().get_accel(i);

        // 检查每个IMU的健康状态
        if (AP::ins().healthy(i)) {
            // 使用该IMU数据
        }
    }
}

// ========== 3. 检测新数据 ==========

bool new_data_available() {
    static uint32_t last_sample_count = 0;
    uint32_t current_count = AP::ins().get_sample_count();

    if (current_count != last_sample_count) {
        last_sample_count = current_count;
        return true;  // 有新数据
    }
    return false;
}
```

#### 4.1.4 支持的IMU芯片

| 芯片型号 | 厂商 | 量程 | 说明 |
|---------|------|------|------|
| MPU6000 | Invensense | ±2000°/s, ±16g | 经典6轴IMU，最常用 |
| MPU6500 | Invensense | ±2000°/s, ±16g | 升级版MPU6000 |
| MPU9250 | Invensense | ±2000°/s, ±16g | 9轴IMU（集成磁力计） |
| ICM20649 | Invensense | ±4000°/s, ±30g | 高性能，运动追踪 |
| ICM20689 | Invensense | ±4000°/s, ±30g | 飞行专用 |
| BMI055 | Bosch | ±2000°/s, ±16g | 汽车级传感器 |
| LSM9DS1 | ST | ±2000°/s, ±16g | 9轴IMU |

#### 4.1.5 IMU数据处理流程

```
传感器数据处理流程：
┌─────────────────────────────────────────────────────────────────┐
│  1. 硬件读取阶段                                                │
│     ├─ 通过SPI/I2C接口读取芯片寄存器                            │
│     ├─ 陀螺仪数据单位: rad/s                                    │
│     └─ 加速度计数据单位: m/s²                                   │
├─────────────────────────────────────────────────────────────────┤
│  2. 温度补偿阶段                                                │
│     ├─ 传感器对温度敏感（温漂）                                  │
│     ├─ 使用内置温度传感器校准                                    │
│     └─ 公式: 校正值 = 原始值 - 温度补偿系数 × 温度差              │
├─────────────────────────────────────────────────────────────────┤
│  3. 零偏校准阶段                                                │
│     ├─ 静止时陀螺仪应有0输出                                    │
│     ├─ 实际存在零点偏移                                          │
│     └─ 公式: 校正值 = 原始值 - 零偏                              │
├─────────────────────────────────────────────────────────────────┤
│  4. 滤波降噪阶段                                                │
│     ├─ 低通滤波器去除高频噪声                                    │
│     ├─ 采样频率 >> 截止频率（通常>10x）                          │
│     └─ 公式: y[n] = α × x[n] + (1-α) × y[n-1]                 │
├─────────────────────────────────────────────────────────────────┤
│  5. EKF融合阶段                                                 │
│     ├─ 与GPS、磁力计等传感器数据融合                             │
│     └─ 输出：姿态角、位置、速度                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 AP_GPS - GPS接收机

**文件位置**: `libraries/AP_GPS/`

GPS提供**位置**、**速度**和**时间**信息，是室外飞行的必需传感器。

#### 4.2.1 GPS数据类结构

```cpp
// Location类 - 地理位置表示
class Location {
public:
    int32_t lat;      // 纬度 × 10^7（度）
    int32_t lng;      // 经度 × 10^7（度）
    int32_t alt;      // 高度（厘米），相对于家点
    int32_t relative_alt;  // 相对高度
    int32_t ground_course; // 地面航向（0.01度）
    uint16_t ground_speed; // 地面速度（cm/s）

    // 转换为度
    double latitude_deg() const { return lat / 1.0e7; }
    double longitude_deg() const { return lng / 1.0e7; }
    float alt_m() const { return alt / 100.0f; }
};
```

#### 4.2.2 GPS类方法

```cpp
class AP_GPS {
public:
    /**
     * GPS定位状态枚举
     */
    enum GPSStatus {
        NO_FIX = 0,        // 无定位，无法使用
        FIX_2D = 1,        // 2D定位（只有水平位置）
        FIX_3D = 2,        // 3D定位（有高度）
        FIX_3D_DGPS = 3,   // 3D + 差分GPS (DGPS)
        FIX_3D_RTK = 4,    // 3D + RTK (厘米级精度)
    };

    /**
     * 获取GPS状态
     */
    GPSStatus status(uint8_t instance=0) const;

    /**
     * 获取地理位置
     */
    const Location& location(uint8_t instance=0) const;

    /**
     * 获取速度向量 (NED坐标系)
     * @return 速度向量，单位：m/s
     */
    const Vector3f& velocity(uint8_t instance=0) const;

    /**
     * 获取水平速度精度 (HDOP)
     * 值越小精度越高
     */
    float get_hdop(uint8_t instance=0) const;

    /**
     * 获取定位精度
     * @return 精度估计（米）
     */
    float get_accuracy(uint8_t instance=0) const;

    /**
     * 获取GPS时间
     */
    uint64_t time_week_ms(uint8_t instance=0) const;
    uint16_t week(uint8_t instance=0) const;

    /**
     * 获取水平速度
     */
    float ground_speed(uint8_t instance=0) const;

    /**
     * 获取航向角（地面航向）
     */
    float ground_course(uint8_t instance=0) const;

    /**
     * 获取可见卫星数量
     */
    uint8_t num_sats(uint8_t instance=0) const;
};
```

#### 4.2.3 GPS使用示例

```cpp
// ========== 1. 基本使用 ==========

void gps_example() {
    AP_GPS *gps = AP::gps();

    // 检查GPS状态
    GPSStatus fix_status = gps->status();

    if (fix_status >= GPS_FIX_3D) {
        // GPS定位有效
        const Location &loc = gps->location();

        float lat = loc.lat / 1.0e7;      // 转换为度
        float lon = loc.lng / 1.0e7;
        float alt = loc.alt / 100.0f;     // 转换为米

        hal.console->printf("GPS: %.7f, %.7f, %.1fm\n", lat, lon, alt);

        // 获取速度
        Vector3f vel = gps->velocity();
        float speed = vel.length();       // 水平速度 m/s
        float climb_rate = -vel.z;         // 上升率 m/s

        // 获取精度
        float accuracy = gps->get_accuracy();
        uint8_t satellites = gps->num_sats();

        hal.console->printf("Speed: %.1f m/s, Sats: %d, Accuracy: %.1fm\n",
                           speed, satellites, accuracy);
    } else {
        // GPS未定位
        hal.console->printf("GPS not fixed: %d\n", (int)fix_status);
    }
}

// ========== 2. GPS时间同步 ==========

void gps_time_example() {
    AP_GPS *gps = AP::gps();

    if (gps->status() >= GPS_FIX_3D) {
        uint64_t time_ms = gps->time_week_ms();
        uint16_t week = gps->week();

        // GPS周秒转换为UNIX时间戳（需要查表或计算GPS起始时间）
        // GPS起始时间: 1980年1月6日 00:00:00 UTC

        hal.console->printf("GPS Time: Week %u, %llu ms\n",
                           (unsigned)week, (unsigned long long)time_ms);
    }
}

// ========== 3. 多GPS冗余 ==========

void multi_gps_example() {
    AP_GPS *gps = AP::gps();
    uint8_t num_gps = gps->num_sensors();

    for (uint8_t i = 0; i < num_gps; i++) {
        if (gps->status(i) >= GPS_FIX_3D) {
            const Location &loc = gps->location(i);
            // 使用第i个GPS
        }
    }
}
```

#### 4.2.4 支持的GPS协议

| 协议 | 说明 | 常见模块 |
|------|------|----------|
| u-blox | 最常用，支持多星系统 | u-blox M8N, M9N, F9P |
| NMEA | 标准NMEA0183协议 | 通用GPS |
| SBF | Septentrio格式 | Septentrio |
| SBP | SwiftNav SBP | Piksi |
| DroneCAN | UAVCAN/CAN总线GPS | CUAV CAN GPS |
| MAVLink | 通过MAVLink获取 | 外部定位系统 |

#### 4.2.5 GPS数据精度对比

| 定位类型 | 精度 | 用途 |
|---------|------|------|
| 标准GPS | 2-5m | 基础飞行 |
| SBAS增强 | 1-2m | 航拍 |
| DGPS | 0.5-1m | 测绘 |
| RTK | 1-2cm | 精准农业、测绘 |

### 4.3 AP_Baro - 气压计

**文件位置**: `libraries/AP_Baro/`

气压计测量**大气压力**，通过气压-高度公式转换为高度。

#### 4.3.1 气压高度原理

```
大气压力与高度的关系（简化模型）：
P = P₀ × (1 - 0.0065 × h / T₀)^(5.257)

其中：
- P = 测量到的气压 (Pa)
- P₀ = 海平面气压 (101325 Pa)
- h = 高度 (m)
- T₀ = 海平面温度 (288.15 K)
- 0.0065 = 温度递减率 (K/m)

简化公式（低高度）：
h ≈ 44330 × (1 - (P/P₀)^0.1903)

ArduPilot使用查表法计算，精度更高
```

#### 4.3.2 气压计类方法

```cpp
class AP_Baro {
public:
    /**
     * 获取压力
     * @return 压力值，单位：帕斯卡 (Pa)
     */
    float get_pressure(uint8_t instance=0) const;

    /**
     * 获取温度
     * @return 温度值，单位：摄氏度
     */
    float get_temperature(uint8_t instance=0) const;

    /**
     * 获取相对于地面的高度
     * @return 高度值，单位：米
     */
    float get_altitude(uint8_t instance=0) const;

    /**
     * 获取海拔高度（绝对高度）
     * @param ground_pressure 地面气压
     * @return 海拔高度（米）
     */
    float get_altitude_difference(uint8_t instance=0, float ground_pressure=101325.0f) const;

    /**
     * 获取滤波后的高度（推荐使用）
     */
    float get_filtered_altitude(uint8_t instance=0) const;
};
```

#### 4.3.3 使用示例

```cpp
void barometer_example() {
    AP_Baro *baro = AP::barometer();

    // 获取数据
    float pressure = baro->get_pressure();       // Pa
    float temperature = baro->get_temperature();   // °C
    float altitude = baro->get_altitude();        // m

    hal.console->printf("Baro: %.0f Pa, %.1f°C, %.1fm\n",
                        pressure, temperature, altitude);

    // 高度变化率（爬升率）
    static float last_alt = 0;
    static uint32_t last_time = 0;

    uint32_t now = AP_HAL::millis();
    if (now != last_time) {
        float climb_rate = (altitude - last_alt) / ((now - last_time) / 1000.0f);
        last_alt = altitude;
        last_time = now;

        hal.console->printf("Climb rate: %.2f m/s\n", climb_rate);
    }
}
```

#### 4.3.4 支持的气压计芯片

| 芯片型号 | 精度 | 说明 |
|---------|------|------|
| MS5611 | 10cm | 高精度，最常用 |
| BMP280 | 1m | 低成本 |
| BMP388 | 50cm | 替代BMP280 |
| ICM20789 | - | 集成在ICM IMU中 |
| DPS280 | 30cm | Bosch新一代 |

### 4.4 AP_Compass - 磁力计

**文件位置**: `libraries/AP_Compass/`

磁力计测量**地球磁场**，提供**航向角**（偏航角）。

#### 4.4.1 地球磁场

```
地球磁场矢量：
        北
         ↑
         │ ← 磁场水平分量
         │
    ┌────┼────┐
    │    │    │
    │    ╰────┼──→ 东
    │         │
    └─────────┘

地球磁场强度：约25-65微特斯拉（μT）
磁场方向：水平分量指向磁北（与真北有磁偏角）
```

#### 4.4.2 磁力计类方法

```cpp
class Compass {
public:
    /**
     * 获取磁场数据（原始）
     * @return 磁场向量，单位：微特斯拉 (μT)
     */
    const Vector3f& get_field(uint8_t instance=0) const;

    /**
     * 获取磁场强度（标量）
     */
    float get_length(uint8_t instance=0) const;

    /**
     * 获取航向角
     * @return 航向角（0-360度），0=北，90=东
     */
    float get_heading(uint8_t instance=0) const;

    /**
     * 获取校准后的磁场数据
     */
    const Vector3f& get_field_calibrated(uint8_t instance=0) const;

    /**
     * 磁力计是否健康
     */
    bool healthy(uint8_t instance=0) const;

    /**
     * 是否使用磁力计
     */
    bool use_for_yaw(uint8_t instance=0) const;
};
```

#### 4.4.3 使用示例

```cpp
void compass_example() {
    Compass *compass = AP::compass();

    // 获取磁场数据
    Vector3f mag = compass->get_field();
    float heading = compass->get_heading();

    hal.console->printf("Mag: %.1f, %.1f, %.1f μT\n",
                        mag.x, mag.y, mag.z);
    hal.console->printf("Heading: %.1f°\n", heading);

    // 磁偏角校正
    // 获取磁场强度
    float strength = compass->get_length();
    hal.console->printf("Field strength: %.1f μT\n", strength);
}
```

#### 4.4.4 磁力计校准

```cpp
/**
 * 磁力计校准是必需的！
 * 因为：
 * 1. 传感器存在零偏
 * 2. 周围金属/磁场干扰
 * 3. 传感器安装角度误差
 *
 * 校准方法：
 * 1. 水平旋转飞行器360度
 * 2. 倾斜飞行器，记录各角度数据
 * 3. 拟合椭圆球体，找到中心偏移和缩放
 * 4. 应用校准参数
 */
```

---

## 5. 姿态估计与导航

### 5.1 AP_AHRS - 姿态航向参考系统

**文件位置**: `libraries/AP_AHRS/`

AHRS是飞控的"**大脑**"，融合所有传感器数据，估计飞行器的**姿态**和**位置**。

#### 5.1.1 AHRS类结构

```cpp
class AP_AHRS {
public:
    // ========== 姿态角获取 ==========

    /**
     * 获取欧拉角（弧度）
     */
    float get_roll_rad() const { return roll; }
    float get_pitch_rad() const { return pitch; }
    float get_yaw_rad() const { return yaw; }

    /**
     * 获取欧拉角（度）- 最常用
     */
    float get_roll_deg() const { return rpy_deg[0]; }
    float get_pitch_deg() const { return rpy_deg[1]; }
    float get_yaw_deg() const { return rpy_deg[2]; }

    /**
     * 获取传感器三角函数值（用于快速计算）
     */
    float cos_roll() const { return _cos_roll; }
    float cos_pitch() const { return _cos_pitch; }
    float cos_yaw() const { return _cos_yaw; }
    float sin_roll() const { return _sin_roll; }
    float sin_pitch() const { return _sin_pitch; }
    float sin_yaw() const { return _sin_yaw; }

    // ========== 旋转矩阵和四元数 ==========

    /**
     * 获取旋转矩阵（NED → 机体）
     */
    const Matrix3f& get_rotation_body_to_ned(void) const;

    /**
     * 获取四元数（推荐，优于欧拉角）
     */
    void get_quat_body_to_ned(Quaternion &quat) const;

    // ========== 传感器数据 ==========

    /**
     * 获取校正后的陀螺仪数据
     */
    const Vector3f& get_gyro(void) const { return state.gyro_estimate; }

    /**
     * 获取地球坐标系下的加速度
     */
    const Vector3f& get_accel_ef() const { return state.accel_ef; }

    /**
     * 获取陀螺仪漂移
     */
    const Vector3f& get_gyro_drift(void) const { return state.gyro_drift; }

    // ========== 位置和速度 ==========

    /**
     * 获取当前位置
     */
    bool get_location(Location &loc) const;

    /**
     * 获取NED速度
     */
    bool get_velocity_NED(Vector3f &vec) const;

    /**
     * 获取相对于家的位置
     */
    bool get_relative_position_NED_home(Vector3f &vec) const;

    // ========== 风估计 ==========

    /**
     * 获取风速估计
     */
    const Vector3f& wind_estimate() const { return state.wind_estimate; }
};
```

#### 5.1.2 欧拉角详解

```
欧拉角定义（Z-Y-X顺序，航空常用）：

    Roll (横滚角) φ:
    - 绕X轴旋转
    - 正值：右翼向下
    - 范围：-180° 到 180°

    Pitch (俯仰角) θ:
    - 绕Y轴旋转
    - 正值：机头向上
    - 范围：-90° 到 90°

    Yaw (偏航角) ψ:
    - 绕Z轴旋转
    - 正值：机头向左（顺时针看）
    - 范围：-180° 到 180°

坐标系约定（NED）：
    北 (X) → 前
    东 (Y) → 右
    下 (Z) → 上方为负
```

#### 5.1.3 AHRS使用示例

```cpp
// ========== 1. 获取姿态数据 ==========

void ahrs_example() {
    AP_AHRS &ahrs = AP::ahrs();

    // 获取欧拉角（度）
    float roll = ahrs.get_roll_deg();
    float pitch = ahrs.get_pitch_deg();
    float yaw = ahrs.get_yaw_deg();

    hal.console->printf("Attitude: R=%.1f°, P=%.1f°, Y=%.1f°\n",
                        roll, pitch, yaw);

    // 获取旋转矩阵
    Matrix3f rotation = ahrs.get_rotation_body_to_ned();

    // 机体到地球坐标转换
    Vector3f body_accel = {1.0f, 0.0f, 0.0f};  // 机体前方1g
    Vector3f earth_accel = ahrs.get_rotation_body_to_ned() * body_accel;

    // 获取四元数
    Quaternion quat;
    ahrs.get_quat_body_to_ned(quat);

    // ========== 2. 获取速度和位置 ==========

    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        float speed = velocity.length();
        float climb_rate = -velocity.z;  // NED坐标系中Z向下

        hal.console->printf("Speed: %.1f m/s, Climb: %.2f m/s\n",
                           speed, climb_rate);
    }

    Location loc;
    if (ahrs.get_location(loc)) {
        hal.console->printf("Position: %.7f, %.7f, %.1fm\n",
                           loc.lat / 1.0e7, loc.lng / 1.0e7, loc.alt / 100.0f);
    }

    // ========== 3. 获取风估计 ==========

    Vector3f wind = ahrs.wind_estimate();
    if (wind.length() > 0.5f) {  // 风速大于0.5 m/s
        hal.console->printf("Wind: %.1f m/s from %.0f°\n",
                           wind.length(), degrees(atan2f(wind.y, wind.x)));
    }
}

// ========== 4. 坐标转换 ==========

void coordinate_transform_example() {
    AP_AHRS &ahrs = AP::ahrs();

    // 机体坐标 → 地球坐标 (NED)
    Vector3f body_vec = {1.0f, 0.0f, 0.0f};  // 机头方向
    Vector3f earth_vec = ahrs.body_to_earth(body_vec);

    // 地球坐标 → 机体坐标
    Vector3f ned_vec = {1.0f, 0.0f, 0.0f};   // 北方
    Vector3f body_result = ahrs.earth_to_body(ned_vec);
}
```

#### 5.1.4 AHRS后端实现

```cpp
enum class EKFType : uint8_t {
    DCM = 0,           // 方向余弦矩阵（简单、可靠）
    TWO = 2,          // EKF2（扩展卡尔曼滤波v2，旧版本）
    THREE = 3,        // EKF3（扩展卡尔曼滤波v3，**推荐**）
    SIM = 10,         // SITL仿真专用
    EXTERNAL = 11,    // 外部姿态估计器（如ROS）
};

/**
 * 选择AHRS后端的考虑因素：
 *
 * DCM:
 * + 简单可靠，计算量小
 * + 无GPS也可工作
 * - 精度不如EKF
 * - 长距离飞行会有漂移
 *
 * EKF3:
 * + 融合多传感器，精度高
 * + 自动检测和拒绝异常数据
 * + 支持多种传感器源
 * - 计算量大
 * - 需要GPS（室外）或光流/视觉（室内）
 */
```

### 5.2 EKF3 - 扩展卡尔曼滤波

**文件位置**: `libraries/AP_NavEKF3/`

EKF3是ArduPilot**推荐使用**的导航滤波器，使用贝叶斯估计融合多传感器数据。

#### 5.2.1 卡尔曼滤波原理

```
卡尔曼滤波核心思想：
基于【预测】（基于模型）+【更新】（基于测量）来估计系统状态

预测阶段（Predict）：
    x̂ₙ₋₁ = F × xₙ₋₁ + B × uₙ₋₁     // 状态预测
    Pₙ₋₁ = F × Pₙ₋₁ × Fᵀ + Q         // 协方差预测

更新阶段（Update）：
    K = Pₙ₋₁ × Hᵀ × (H × Pₙ₋₁ × Hᵀ + R)⁻¹   // 卡尔曼增益
    x̂ₙ = x̂ₙ₋₁ + K × (z - H × x̂ₙ₋₁)          // 状态更新
    Pₙ = (I - K × H) × Pₙ₋₁                  // 协方差更新

其中：
- x̂: 状态估计
- P: 估计协方差（不确定性）
- F: 状态转移矩阵
- H: 观测矩阵
- Q: 过程噪声
- R: 测量噪声
- K: 卡尔曼增益
- z: 测量值
```

#### 5.2.2 EKF3状态向量

```cpp
/**
 * EKF3估计的状态量：
 */
struct EKFState {
    // ===== 位置 (3) =====
    float north;           // 北向位置 (m)
    float east;            // 东向位置 (m)
    float down;            // 高度 (m，正值向下)

    // ===== 速度 (3) =====
    float vel_north;      // 北向速度 (m/s)
    float vel_east;       // 东向速度 (m/s)
    float vel_down;       // 垂直速度 (m/s)

    // ===== 姿态 (4) =====
    // 使用四元数表示 (4个变量，但只有3个自由度)
    Quaternion attitude;   // 姿态四元数

    // ===== 传感器偏差 (6) =====
    Vector3f gyro_bias;    // 陀螺仪零偏 (rad/s)
    Vector3f accel_bias;  // 加速度计零偏 (m/s²)

    // ===== 磁场 (3，可选）=====
    Vector3f mag_bias;     // 磁力计零偏

    // ===== 风速 (2，可选）=====
    float wind_north;      // 北向风速 (m/s)
    float wind_east;       // 东向风速 (m/s)
};

/**
 * 总计：最多21个状态变量
 * 状态向量维度影响计算量
 */
```

#### 5.2.3 EKF3传感器融合

```cpp
/**
 * EKF3传感器融合流程：
 */

// 预测步骤（高频，400Hz）：
void ekf_predict(float gyro_rad[], float accel_m_s2[], float dt) {
    // 1. 使用陀螺仪积分更新姿态
    // 2. 使用加速度计（去除重力后）更新速度
    // 3. 积分速度更新位置
    // 4. 估计传感器偏差
}

// 更新步骤（低频，各传感器频率不同）：

// GPS位置更新（5-20Hz）：
void ekf_update_gps(float lat, float lon, float alt, float timestamp) {
    // 将GPS位置与EKF预测位置比较
    // 计算创新（残差）
    // 根据卡尔曼增益更新状态
}

// GPS速度更新（5-20Hz）：
void ekf_update_gps_vel(float v_north, float v_east, float v_down) {
    // 融合GPS速度，提高速度估计精度
}

// 气压计高度更新（10-50Hz）：
void ekf_update_baro(float altitude) {
    // 融合气压高度
    // 注意：气压高度有漂移，需要与GPS融合
}

// 磁力计航向更新（5-20Hz）：
void ekf_update_mag(float mag_x, float mag_y, float mag_z) {
    // 融合磁力计航向
    // 自动检测和校正磁场干扰
}

// 光流/视觉更新（高速）：
void ekf_update_optical_flow(float flow_x, float flow_y) {
    // 在GPS信号弱时使用
    // 需要已知高度
}
```

#### 5.2.4 EKF3诊断

```cpp
// ========== 1. 检查滤波器状态 ==========

void check_ekf_status() {
    AP_AHRS &ahrs = AP::ahrs();
    nav_filter_status status;

    ahrs.get_filter_status(status);

    if (status.flags.attitude) {
        // 姿态估计有效
    }
    if (status.flags.horiz_pos) {
        // 水平位置估计有效
    }
    if (status.flags.vert_pos) {
        // 垂直位置估计有效
    }
    if (status.flags.const_pos_mode) {
        // 处于恒定位置模式（无GPS）
    }
}

// ========== 2. 获取创新（残差）==========

void check_innovations() {
    Vector3f vel_innov, pos_innov, mag_innov;
    float tas_innov, yaw_innov;

    AP::ahrs().get_innovations(vel_innov, pos_innov, mag_innov,
                               tas_innov, yaw_innov);

    // 创新值应该接近0
    // 如果创新值过大，说明传感器有问题或EKF有问题
    if (vel_innov.length() > 1.0f) {
        // 速度创新异常
    }
}

// ========== 3. 获取方差（不确定性）==========

void check_variances() {
    float vel_var, pos_var, hgt_var;
    Vector3f mag_var;
    float tas_var;

    AP::ahrs().get_variances(vel_var, pos_var, hgt_var, mag_var, tas_var);

    // 方差值越小，估计越可靠
    // 方差 > 1.0 表示估计不可靠
    if (pos_var > 1.0f) {
        // 位置估计不可靠
    }
}
```

### 5.3 DCM - 方向余弦矩阵

**文件位置**: `libraries/AP_AHRS/AP_AHRS_DCM.cpp`

DCM是最**基础**的姿态估计算法，原理简单、计算量小。

#### 5.3.1 DCM原理

```
DCM表示从机体坐标系到地球坐标系的旋转矩阵：

DCM = [ X_body  →  NED ]
      [ Y_body  →  NED ]
      [ Z_body  →  NED ]

DCM矩阵示例（水平飞行，机头朝北）：
      [ 1    0    0  ]     // X轴（机头）指向北
      [ 0    1    0  ]     // Y轴（右侧）指向东
      [ 0    0   -1  ]     // Z轴（向下）

更新公式：
DCM_new = DCM_old + DCM_old × (ω × dt)

其中ω是角速度向量，dt是时间步长

从DCM提取欧拉角：
roll  = atan2(DCM[1][2], DCM[2][2])
pitch = -asin(DCM[0][2])
yaw   = atan2(DCM[0][1], DCM[0][0])
```

#### 5.3.2 DCM vs EKF

| 特性 | DCM | EKF3 |
|------|-----|------|
| 计算量 | 小 | 大 |
| 内存占用 | 小 | 大 |
| 精度 | 一般 | 高 |
| 传感器融合 | 仅陀螺仪+加速度计 | 多传感器融合 |
| 异常检测 | 无 | 有 |
| 适用场景 | 简单飞行、室内 | 室外长距离、精准作业 |

---

## 6. 控制系统

### 6.1 AC_AttitudeControl - 姿态控制

**文件位置**: `libraries/AC_AttitudeControl/`

姿态控制器根据**目标姿态**计算**电机输出**。

#### 6.1.1 控制层级

```
飞控控制层级：
┌─────────────────────────────────────────────────────────────────┐
│  1. 位置控制器 (Position Controller)                            │
│     输入: 目标位置 (x, y, z)                                    │
│     输出: 目标姿态角 + 目标油门                                  │
│     用途: 自动飞行、悬停                                         │
├─────────────────────────────────────────────────────────────────┤
│  2. 姿态控制器 (Attitude Controller)                            │
│     输入: 目标姿态角 (roll, pitch, yaw)                         │
│     输出: 目标角速度 (roll_rate, pitch_rate, yaw_rate)          │
│     算法: P控制器 + 前馈                                         │
├─────────────────────────────────────────────────────────────────┤
│  3. 角速度控制器 (Rate Controller)                              │
│     输入: 目标角速度                                             │
│     输出: 各电机PWM差值                                          │
│     算法: PID控制器                                              │
├─────────────────────────────────────────────────────────────────┤
│  4. 电机混控器 (Motor Mixer)                                    │
│     输入: Roll/Pitch/Yaw/Throttle控制量                          │
│     输出: 各电机PWM值                                           │
└─────────────────────────────────────────────────────────────────┘
```

#### 6.1.2 多旋翼姿态控制类

```cpp
class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
    /**
     * 设置目标姿态（四元数）
     * @param target_attitude 目标四元数
     */
    void set_attitude_target_quaternion(const Quaternion& target_attitude);

    /**
     * 设置目标姿态（欧拉角，度）
     * @param roll_deg  横滚角目标
     * @param pitch_deg 俯仰角目标
     * @param yaw_deg   偏航角目标
     */
    void set_attitude_target_euler(float roll_deg, float pitch_deg, float yaw_deg);

    /**
     * 设置目标角速度（度/秒）
     * 这是rate控制器的输入
     */
    void set_angvel_target(const Vector3f& angvel_rads);

    /**
     * 执行角速度控制（每帧调用）
     * 从set_angvel_target()获取目标，计算电机输出
     */
    void rate_controller_run();

    /**
     * 设置油门输出
     * @param throttle_in  油门输入 (0-1)
     * @param apply_angle_correction 是否应用角度校正
     */
    void set_throttle_out(float throttle_in, bool apply_angle_correction);

    /**
     * 设置悬停油门（平衡点）
     */
    void set_hover_throttle(float throttle) { _hover_throttle = throttle; }
};
```

#### 6.1.3 使用示例

```cpp
// ========== 1. 设置目标姿态 ==========

void attitude_control_example() {
    AC_AttitudeControl *attitude_control = copter.attitude_control;

    // 目标：水平悬停
    attitude_control->set_attitude_target_euler(0.0f, 0.0f, 0.0f);

    // 目标：右倾10度
    attitude_control->set_attitude_target_euler(10.0f, 0.0f, 0.0f);

    // 目标：前倾5度，右转30度
    attitude_control->set_attitude_target_euler(0.0f, 5.0f, 30.0f);
}

// ========== 2. 角速度控制 ==========

void rate_control_example() {
    AC_AttitudeControl *attitude_control = copter.attitude_control;

    // 目标角速度（度/秒）
    Vector3f target_rates;
    target_rates.x = 100.0f;   // 向右滚转
    target_rates.y = 0.0f;
    target_rates.z = 0.0f;

    // 设置目标角速度
    attitude_control->set_angvel_target(target_rates);

    // 执行控制（在fast_loop中每帧调用）
    attitude_control->rate_controller_run();
}

// ========== 3. 设置油门 ==========

void throttle_control_example() {
    AC_AttitudeControl *attitude_control = copter.attitude_control;

    // 设置悬停油门（50%油门平衡飞机重量）
    attitude_control->set_hover_throttle(500);  // PWM值

    // 直接设置油门
    attitude_control->set_throttle_out(500, true);
}
```

### 6.2 PID控制原理详解

```
PID控制器结构图：

    目标值  ┌────────┐
   ───────►│        │     误差     ┌────────┐
           │   P    │──────────────►│        │
    测量值 │        │     P输出    │   求和  │◄──── 测量值
   ◄──────│        │◄──────────────│        │◄─────┐
           └────────┘              │        │       │
                                    │   I    │◄──────┤
           ┌────────┐              │        │       │
   误差 ───►│        │     D输出    │        │◄──────┘
           │   D    │──────────────►│        │
    测量值 │        │◄──────────────│        │◄────── 测量值
   ───────│        │   微分        │        │
           └────────┘              └────────┘
                                        │
                                        ▼
                                   输出到执行器

各参数作用：
┌─────────────────────────────────────────────────────────────┐
│ P (比例)  Proportional                                       │
├─────────────────────────────────────────────────────────────┤
│ 作用：误差越大，输出越大                                     │
│ 公式：output = Kp × error                                   │
│ 优点：响应快                                                │
│ 缺点：可能振荡，无法消除稳态误差                             │
├─────────────────────────────────────────────────────────────┤
│ I (积分)  Integral                                          │
├─────────────────────────────────────────────────────────────┤
│ 作用：累积误差，消除稳态误差                                 │
│ 公式：output = Ki × ∫error dt                               │
│ 优点：消除稳态误差                                          │
│ 缺点：积分饱和、超调                                        │
├─────────────────────────────────────────────────────────────┤
│ D (微分)  Derivative                                        │
├─────────────────────────────────────────────────────────────┤
│ 作用：预测误差趋势，增加阻尼                                 │
│ 公式：output = Kd × d(error)/dt                             │
│ 优点：减少振荡、改善稳定性                                   │
│ 缺点：对噪声敏感                                            │
└─────────────────────────────────────────────────────────────┘

调参口诀：
┌─────────────────────────────────────────────────────────────┐
│ 1. 先调P，找到系统振荡的临界点                               │
│ 2. 再调D，增加阻尼消除振荡                                   │
│ 3. 最后调I，消除稳态误差                                     │
│                                                             │
│ P太大 → 系统振荡                                            │
│ I太小 → 稳态误差（位置不准）                                │
│ D太大 → 噪声敏感、抖动                                      │
└─────────────────────────────────────────────────────────────┘
```

### 6.3 电机混控

```
多旋翼电机混控原理：

四旋翼X模式：
       前
        ↑
       M1
        │
  M2◄──┼──►M4
        │
       M3
        ↓
       后

电机编号（逆时针）：
M1: 前右   (CW)
M2: 后右   (CCW)
M3: 后左   (CCW)
M4: 前左   (CW)

混控公式：
motor1 = throttle + roll + pitch + yaw
motor2 = throttle - roll + pitch - yaw
motor3 = throttle - roll - pitch + yaw
motor4 = throttle + roll - pitch - yaw

电机转向：
CW (顺时针) 产生反扭矩
CCW (逆时针) 产生反扭矩

姿态控制时的电机变化：
- 右滚 (Roll+): M1、M4加速，M2、M3减速 → 产生向右的力矩
- 前俯 (Pitch+): M1、M4减速，M2、M3加速 → 产生向前的力矩
- 右转 (Yaw+): M1、M2加速，M3、M4减速 → 产生顺时针扭矩
```

### 6.4 飞行模式

ArduPilot支持多种飞行模式：

| 模式 | 类型 | 说明 | 适用场景 |
|------|------|------|----------|
| Stabilize | 手动 | 手动控制姿态，油门中立悬停 | 新手练习 |
| Acro | 手动 | 手动控制角速度，油门控制高度 | 特技飞行 |
| AltHold | 半自主 | 高度自动保持，手动水平控制 | 航拍 |
| Loiter | 自主 | 位置自动保持（GPS） | 悬停拍摄 |
| RTL | 自动 | 自动返回起航点 | 紧急返航 |
| Auto | 自动 | 按任务航点飞行 | 自主作业 |
| Guided | 自动 | 地面站引导飞行 | 地面站控制 |
| Circle | 自动 | 绕圈飞行 | 环绕拍摄 |
| PosHold | 半自主 | 简化版位置保持 | 简单悬停 |
| Brake | 自动 | 急停悬停 | 快速停止 |
| FollowMe | 自动 | 跟随移动目标 | 跟随拍摄 |

### 6.5 控制算法数学原理与代码实现

本节深入讲解飞控核心算法的**数学物理模型**、**公式推导**及**代码实现**。

---

#### 6.5.1 PID控制器 - 数学原理与离散实现

##### 6.5.1.1 连续PID控制器公式

PID控制器是飞控中最常用的控制算法，其连续形式的标准公式为：

$$u(t) = K_p \cdot e(t) + K_i \cdot \int_0^t e(\tau)d\tau + K_d \cdot \frac{de(t)}{dt}$$

其中：
- $u(t)$：控制器输出
- $e(t) = r(t) - y(t)$：目标值与测量值的误差
- $K_p$：比例增益
- $K_i$：积分增益
- $K_d$：微分增益

**各分量物理意义：**

| 分量 | 数学表达式 | 物理意义 | 对系统的影响 |
|------|-----------|----------|-------------|
| 比例项 P | $K_p \cdot e$ | 误差的瞬时放大 | 响应速度，与误差成正比 |
| 积分项 I | $K_i \cdot \int e dt$ | 误差的累积 | 消除稳态误差，但可能导致超调 |
| 微分项 D | $K_d \cdot de/dt$ | 误差变化率 | 增加阻尼，抑制振荡 |

##### 6.5.1.2 离散PID控制器（数字实现）

由于飞控使用数字处理器，必须将连续公式离散化。采用**后向欧拉法**进行离散化：

**离散化推导：**

积分项离散化：
$$\int_0^t e(\tau)d\tau \approx \sum_{n=0}^{k} e[n] \cdot \Delta t$$

微分项离散化：
$$\frac{de(t)}{dt} \approx \frac{e[k] - e[k-1]}{\Delta t}$$

**离散PID公式：**
$$u[k] = K_p \cdot e[k] + K_i \cdot \sum_{n=0}^{k} e[n] \cdot \Delta t + K_d \cdot \frac{e[k] - e[k-1]}{\Delta t}$$

其中：
- $k$：当前采样时刻
- $\Delta t$：采样周期（控制周期）

##### 6.5.1.3 ArduPilot PID实现源码解析

**文件位置**: `libraries/PID/PID.cpp`

```cpp
/**
 * PID控制器 - 离散实现
 *
 * 数学公式：
 * u[k] = Kp*e[k] + Ki*Σ(e[n]) + Kd*de/dt
 *
 * 特点：
 * 1. 使用低通滤波平滑微分项（避免噪声放大）
 * 2. 积分限幅防止积分饱和
 * 3. 自动微分计算，处理测量值跳变
 */

#include "PID.h"
#include <AP_HAL/AP_HAL.h>

// PID参数默认值
const AP_Param::GroupInfo PID::var_info[] = {
    AP_PARAMINFO(_kp),
    AP_PARAMINFO(_ki),
    AP_PARAMINFO(_kd),
    AP_PARAMINFO(_kf),
    AP_PARAMINFO(_imax),
    AP_PARAMINFO(_fCut),
    AP_PARAMEND
};

/**
 * 计算PID输出
 *
 * @param error    误差值（目标值 - 测量值）
 * @param scaler    缩放因子（用于多旋翼油门混控）
 * @return         PID输出值
 *
 * 数学流程：
 * 1. 计算P项: P = Kp * error
 * 2. 计算D项: D = Kd * d(error)/dt (低通滤波后)
 * 3. 计算I项: I = Ki * Σ(error) (带限幅)
 * 4. 输出: u = P + I + D
 */
float PID::get_pid(float error, float scaler)
{
    // ========== 1. 时间计算 ==========
    uint32_t tnow = AP_HAL::millis();                    // 当前系统时间（毫秒）
    uint32_t dt = tnow - _last_t;                        // 采样间隔
    float output = 0;                                     // 初始化输出
    float delta_time;                                    // 采样间隔（秒）

    /**
     * 首次运行或长时间中断后的处理：
     * - dt > 1000ms (1秒)：认为系统刚启动，重置积分项
     * - 避免积分项累积过大的初始误差
     */
    if (_last_t == 0 || dt > 1000) {
        dt = 0;
        reset_I();                                       // 重置积分器
    }
    _last_t = tnow;                                      // 更新上次时间
    delta_time = (float)dt * 0.001f;                     // 转换为秒

    // ========== 2. 比例项 (P) ==========
    // 公式: P = Kp * error
    _pid_info.P = error * _kp;                           // 比例分量
    output += _pid_info.P;                               // 加入总输出

    // ========== 3. 微分项 (D) - 带低通滤波 ==========
    /**
     * 微分项的噪声问题：
     * - 微分运算会放大高频噪声
     * - 解决方案：使用一阶低通滤波平滑
     *
     * 一阶低通滤波公式：
     * y[k] = α * x[k] + (1-α) * y[k-1]
     * 其中 α = Δt / (RC + Δt)
     * RC: 时间常数（fCut为截止频率，RC = 1/(2πfCut)）
     */
    if ((fabsf(_kd) > 0) && (dt > 0)) {
        float derivative;                               // 原始微分值
        float RC;                                       // RC时间常数

        // 首次使用：初始化微分值
        if (isnan(_last_derivative)) {
            derivative = 0;
            _last_derivative = 0;
        } else {
            // 原始微分计算（后向差分）
            // de/dt ≈ (e[k] - e[k-1]) / Δt
            derivative = (error - _last_error) / delta_time;
        }

        // 低通滤波平滑微分值
        // 截止频率fCut越高，滤波效果越弱（保留更多高频信号）
        RC = 1.0f / (2.0f * M_PI * _fCut);
        /**
         * 滤波系数推导：
         * α = Δt / (RC + Δt)
         * 当Δt << RC时，α ≈ Δt/RC（小步长平滑）
         * 当Δt >> RC时，α ≈ 1（几乎不滤波）
         */
        derivative = _last_derivative +
                    ((delta_time / (RC + delta_time)) *
                     (derivative - _last_derivative));

        // 存储原始微分值用于下一次迭代
        _last_error = error;
        _last_derivative = derivative;

        // 应用微分增益
        _pid_info.D = derivative * _kd;
        output += _pid_info.D;
    }

    // ========== 4. 积分项 (I) - 带限幅 ==========
    /**
     * 积分项的作用：
     * - 累积误差，消除稳态误差
     * - 对于飞控：即使有持续的外力（如风），也能保持位置
     *
     * 积分饱和问题：
     * - 积分器会不断累积，即使输出已被限制
     * - 解决方法：限制积分项的幅度
     */
    if (fabsf(_ki) > 0) {
        // 积分累加：I = I + Ki * error * Δt
        _integrator += error * delta_time * _ki;

        // 积分限幅
        // 限制积分器输出不超过一定范围，防止积分饱和
        float integrator_max = _imax / fabsf(_ki);
        _integrator = constrain_float(_integrator, -integrator_max, integrator_max);

        _pid_info.I = _integrator;
        output += _integrator;
    }

    // ========== 5. 应用缩放因子 ==========
    /**
     * 缩放因子的作用：
     * - 用于多旋翼的油门混控
     * - 当飞机倾斜时，需要增加油门以维持升力
     * - 缩放因子 = 1/cos(roll) * 1/cos(pitch)
     */
    if (scaler != 1.0f) {
        output *= scaler;
    }

    // 限制最终输出范围
    output = constrain_float(output, -_imax, _imax);

    // 记录本次信息用于调试
    _pid_info.err = error;
    _pid_info.desired = 0;
    _pid_info.FF = 0;

    return output;
}

/**
 * 获取角速度PID输出
 *
 * 与get_pid的区别：
 * 1. 自动微分：不使用误差的微分，而使用测量值的变化率
 * 2. 避免测量噪声放大：微分作用于测量值而非误差
 *
 * 公式：
 * D = Kd * d(measurement)/dt (而非 d(error)/dt)
 */
float PID::get_pid_angvel(float error, float measurement, float scaler)
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t_angvel;
    float output = 0;
    float delta_time;

    // 时间处理（与get_pid相同）
    if (_last_t_angvel == 0 || dt > 1000) {
        dt = 0;
        reset_I();
    }
    _last_t_angvel = tnow;
    delta_time = (float)dt * 0.001f;

    // P项和I项（与get_pid相同）
    _pid_info.P = error * _kp;
    output += _pid_info.P;

    if (fabsf(_ki) > 0) {
        _integrator += error * delta_time * _ki;
        float integrator_max = _imax / fabsf(_ki);
        _integrator = constrain_float(_integrator, -integrator_max, integrator_max);
        _pid_info.I = _integrator;
        output += _integrator;
    }

    // D项：使用测量值微分（自动微分）
    /**
     * 自动微分 vs 手动微分：
     * - 手动微分：d(error)/dt = (target - measurement)/dt
     * - 自动微分：d(measurement)/dt
     *
     * 自动微分的优势：
     * - 目标值变化时项不会产生，D冲击响应
     * - 更平滑的控制效果
     */
    if ((fabsf(_kd) > 0) && (dt > 0)) {
        float derivative;

        if (isnan(_last_derivative_angvel)) {
            derivative = 0;
            _last_derivative_angvel = 0;
        } else {
            // 测量值微分（而非误差微分）
            derivative = (measurement - _last_measurement) / delta_time;
        }

        float RC = 1.0f / (2.0f * M_PI * _fCut);
        derivative = _last_derivative_angvel +
                    ((delta_time / (RC + delta_time)) *
                     (derivative - _last_derivative_angvel));

        _last_measurement = measurement;
        _last_derivative_angvel = derivative;

        _pid_info.D = derivative * _kd;
        output += _pid_info.D;
    }

    if (scaler != 1.0f) {
        output *= scaler;
    }

    output = constrain_float(output, -_imax, _imax);

    _pid_info.err = error;
    _pid_info.desired = 0;

    return output;
}

/**
 * 重置积分器
 *
 * 使用场景：
 * 1. 控制器切换模式
 * 2. 检测到异常状态
 * 3. 手动重置
 */
void PID::reset_I()
{
    _integrator = 0;
    _last_derivative = NAN;      // 标记为未初始化
    _last_derivative_angvel = NAN;
}
```

##### 6.5.1.4 PID参数调谐指南

**调参原则：**

```
调参顺序：P → D → I

┌─────────────────────────────────────────────────────────────┐
│ 第一步：调节比例增益Kp                                      │
├─────────────────────────────────────────────────────────────┤
│ 1. 设置 Ki = 0, Kd = 0                                      │
│ 2. 逐渐增大Kp直到：                                         │
│    - 系统开始振荡（临界增益）                                │
│    - 记录此时的Kp值                                         │
│ 3. 设置工作Kp = 0.5 ~ 0.7 × 临界增益                        │
│                                                              │
│ 判断标准：                                                  │
│ - Kp太小：响应慢，误差消除慢                                │
│ - Kp合适：快速响应，轻微超调                                │
│ - Kp太大：持续振荡，无法稳定                                │
├─────────────────────────────────────────────────────────────┤
│ 第二步：调节微分增益Kd                                      │
├─────────────────────────────────────────────────────────────┤
│ 1. 在合适Kp基础上，逐渐增大Kd                               │
│ 2. 目标：消除振荡，增加阻尼                                 │
│                                                              │
│ 判断标准：                                                  │
│ - Kd太小：仍有振荡                                          │
│ - Kd合适：振荡快速衰减                                      │
│ - Kd太大：响应迟缓，抖动                                    │
├─────────────────────────────────────────────────────────────┤
│ 第三步：调节积分增益Ki                                      │
├─────────────────────────────────────────────────────────────┤
│ 1. 逐渐增大Ki直到稳态误差可接受                             │
│ 2. 观察：是否出现超调增大或积分饱和现象                     │
│                                                              │
│ 判断标准：                                                  │
│ - Ki太小：稳态误差无法消除                                  │
│ - Ki合适：稳态误差快速消除                                  │
│ - Ki太大：超调增大，可能振荡                                │
└─────────────────────────────────────────────────────────────┘
```

**多旋翼典型PID参数范围：**

```cpp
// 角速度环PID参数（AC_AttitudeControl中）
// Roll/Pitch角速度环
// Kp ≈ 0.15 ~ 0.25 (单位：rad/s per rad)
// Ki ≈ 0.0 ~ 0.02
// Kd ≈ 0.0 ~ 0.5 (实际使用自动微分)

#define ATC_RAT_RLL_P               0.15f  // Roll角速度P
#define ATC_RAT_RLL_I               0.0f   // Roll角速度I
#define ATC_RAT_RLL_D               0.0f   // Roll角速度D（使用自动微分）
#define ATC_RAT_RLL_IMAX            0.0f   // Roll角速度积分限幅

#define ATC_RAT_PIT_P               0.15f  // Pitch角速度P
#define ATC_RAT_PIT_I               0.0f   // Pitch角速度I
#define ATC_RAT_PIT_D               0.0f
#define ATC_RAT_PIT_IMAX            0.0f

// Yaw角速度环（需要更高增益，因为扭矩常数较小）
#define ATC_RAT_YAW_P               0.20f  // Yaw角速度P
#define ATC_RAT_YAW_I               0.02f  // Yaw角速度I（需要消除偏航漂移）
#define ATC_RAT_YAW_D               0.0f
#define ATC_RAT_YAW_IMAX            0.0f
```

---

#### 6.5.2 四元数 - 3D旋转的数学原理

##### 6.5.2.1 为什么使用四元数

欧拉角的万向节锁问题：

```
欧拉角旋转顺序：Z → Y → X

当 Pitch = ±90° 时：
- Roll轴和Yaw轴绕同一轴旋转
- 失去一个自由度
- 无法表示某些旋转

四元数优势：
1. 任意轴旋转，无万向节锁
2. 球面插值（Slerp）平滑
3. 计算效率高（4个元素 vs 9个矩阵元素）
4. 数值稳定性好
```

##### 6.5.2.2 四元数数学定义

**四元数定义：**
$$q = q_0 + q_1\mathbf{i} + q_2\mathbf{j} + q_3\mathbf{k}$$

其中虚数单位满足：
$$\mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = \mathbf{ijk} = -1$$
$$\mathbf{ij} = \mathbf{k}, \quad \mathbf{ji} = -\mathbf{k}$$
$$\mathbf{jk} = \mathbf{i}, \quad \mathbf{kj} = -\mathbf{i}$$
$$\mathbf{ki} = \mathbf{j}, \quad \mathbf{ik} = -\mathbf{j}$$

**四元数表示旋转：**
$$q = \begin{bmatrix} q_0 \\ \vec{q} \end{bmatrix} = \begin{bmatrix} \cos(\theta/2) \\ \vec{n}\sin(\theta/2) \end{bmatrix}$$

其中：
- $\theta$：绕轴旋转角度
- $\vec{n}$：单位旋转轴向量

**模长约束（单位四元数）：**
$$|q| = q_0^2 + q_1^2 + q_2^2 + q_3^2 = 1$$

##### 6.5.2.3 四元数乘法（旋转合成）

**Hamilton积：**
$$q_1 \otimes q_2 = \begin{bmatrix} q_{10}q_{20} - \vec{q}_1 \cdot \vec{q}_2 \\ q_{10}\vec{q}_2 + q_{20}\vec{q}_1 + \vec{q}_1 \times \vec{q}_2 \end{bmatrix}$$

**旋转合成公式：**
若 $q_1$ 表示旋转A，$q_2$ 表示旋转B，则：
$$q_{total} = q_2 \otimes q_1$$

表示先应用A，再应用B（注意顺序！）。

##### 6.5.2.4 四元数旋转向量

**向量旋转公式：**
给定单位四元数 $q$ 和向量 $\vec{v}$，旋转后的向量：

$$v' = q \otimes \begin{bmatrix} 0 \\ \vec{v} \end{bmatrix} \otimes q^*$$

其中共轭四元数 $q^* = \begin{bmatrix} q_0 \\ -\vec{q} \end{bmatrix}$

展开公式：
$$
\vec{v}' = \vec{v} + 2q_0(\vec{q} \times \vec{v}) + 2(\vec{q} \times (\vec{q} \times \vec{v}))
$$

##### 6.5.2.5 四元数与欧拉角转换

**欧拉角转四元数（Z-Y-X顺序）：**

$$
q_0 = \cos(\phi/2)\cos(\theta/2)\cos(\psi/2) + \sin(\phi/2)\sin(\theta/2)\sin(\psi/2)
$$
$$
q_1 = \sin(\phi/2)\cos(\theta/2)\cos(\psi/2) - \cos(\phi/2)\sin(\theta/2)\sin(\psi/2)
$$
$$
q_2 = \cos(\phi/2)\sin(\theta/2)\cos(\psi/2) + \sin(\phi/2)\cos(\theta/2)\sin(\psi/2)
$$
$$
q_3 = \cos(\phi/2)\cos(\theta/2)\sin(\psi/2) - \sin(\phi/2)\sin(\theta/2)\cos(\psi/2)
$$

**四元数转欧拉角（Z-Y-X顺序）：**

$$
\phi = \arctan2(2(q_0q_1 + q_2q_3), 1 - 2(q_1^2 + q_2^2))
$$
$$
\theta = \arcsin(2(q_0q_2 - q_3q_1))
$$
$$
\psi = \arctan2(2(q_0q_3 + q_1q_2), 1 - 2(q_2^2 + q_3^2))
$$

##### 6.5.2.6 ArduPilot四元数代码实现

**文件位置**: `libraries/AP_Math/quaternion.h`

```cpp
/**
 * 四元数类 - 3D旋转的高效表示
 *
 * 数学基础：
 * q = [w, x, y, z] = [cos(θ/2), n_x*sin(θ/2), n_y*sin(θ/2), n_z*sin(θ/2)]
 * 其中 n = [n_x, n_y, n_z] 是单位旋转轴，θ 是旋转角度
 */

#pragma once

#include <AP_Math/AP_Math.h>

class Matrix3f;

/**
 * 四元数类定义
 *
 * 数据成员：
 * q0: 实部 (scalar part)
 * q1, q2, q3: 虚部 (vector part)
 */
class Quaternion {
public:
    float q0, q1, q2, q3;        // 四元数四个分量

    /**
     * 构造函数 - 初始化为单位四元数（无旋转）
     */
    Quaternion() :
        q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}

    /**
     * 构造函数 - 直接指定四元数分量
     */
    Quaternion(float _q0, float _q1, float _q2, float _q3) :
        q0(_q0), q1(_q1), q2(_q2), q3(_q3) {}

    /**
     * 从欧拉角构造四元数
     *
     * 数学公式：Z-Y-X旋转顺序
     * 先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll
     *
     * @param roll_deg  横滚角（度）
     * @param pitch_deg 俯仰角（度）
     * @param yaw_deg   偏航角（度）
     */
    void from_euler(float roll_deg, float pitch_deg, float yaw_deg)
    {
        // 转换为弧度
        float roll_rad = radians(roll_deg);
        float pitch_rad = radians(pitch_deg);
        float yaw_rad = radians(yaw_deg);

        // 计算半角
        float cr = cosf(roll_rad * 0.5f);
        float cp = cosf(pitch_rad * 0.5f);
        float cy = cosf(yaw_rad * 0.5f);

        float sr = sinf(roll_rad * 0.5f);
        float sp = sinf(pitch_rad * 0.5f);
        float sy = sinf(yaw_rad * 0.5f);

        // 四元数计算公式（Z-Y-X顺序）
        // q = q_yaw ⊗ q_pitch ⊗ q_roll
        q0 = cr * cp * cy + sr * sp * sy;
        q1 = sr * cp * cy - cr * sp * sy;
        q2 = cr * sp * cy + sr * cp * sy;
        q3 = cr * cp * sy - sr * sp * cy;
    }

    /**
     * 将四元数转换为欧拉角
     *
     * 数学公式：
     * roll  = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
     * pitch = asin(2(q0q2 - q3q1))
     * yaw   = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
     *
     * @return  Vector3f(x=roll, y=pitch, z=yaw) 单位：度
     */
    Vector3f to_euler() const
    {
        Vector3f euler;

        // Roll (X轴旋转)
        /**
         * 推导：
         * 从四元数到旋转矩阵的第三列：
         * R = [r11 r12 r13]
         *     [r21 r22 r23]
         *     [r31 r32 r33]
         * roll = atan2(r32, r33)
         *      = atan2(2(q0q1 + q2q3), q0² + q3² - q1² - q2²)
         *      = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
         */
        euler.x = degrees(atan2f(2.0f * (q0 * q1 + q2 * q3),
                                 1.0f - 2.0f * (q1 * q1 + q2 * q2)));

        // Pitch (Y轴旋转)
        /**
         * 推导：
         * pitch = asin(r31)
         *       = asin(2(q0q2 - q3q1))
         */
        euler.y = degrees(asinf(2.0f * (q0 * q2 - q3 * q1)));

        // Yaw (Z轴旋转)
        /**
         * 推导：
         * yaw = atan2(r21, r11)
         *     = atan2(2(q0q3 + q1q2), q0² + q1² - q2² - q3²)
         *     = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
         */
        euler.z = degrees(atan2f(2.0f * (q0 * q3 + q1 * q2),
                                 1.0f - 2.0f * (q2 * q2 + q3 * q3)));

        return euler;
    }

    /**
     * 四元数乘法 - 旋转合成
     *
     * 数学公式：
     * q1 ⊗ q2 = [q1_0*q2_0 - q1·q2, q1_0*q2 + q2_0*q1 + q1×q2]
     *
     * 使用场景：
     * 1. 连续旋转合成
     * 2. 角速度积分（dq/dt = 0.5 * ω ⊗ q）
     *
     * @param v  右侧四元数
     * @return   乘积四元数
     */
    Quaternion operator*(const Quaternion &v) const
    {
        Quaternion result;

        // Hamilton积展开
        result.q0 = q0 * v.q0 - q1 * v.q1 - q2 * v.q2 - q3 * v.q3;
        result.q1 = q0 * v.q1 + q1 * v.q0 + q2 * v.q3 - q3 * v.q2;
        result.q2 = q0 * v.q2 - q1 * v.q3 + q2 * v.q0 + q3 * v.q1;
        result.q3 = q0 * v.q3 + q1 * v.q2 - q2 * v.q1 + q3 * v.q0;

        return result;
    }

    /**
     * 从轴角构造四元数
     *
     * 数学公式：
     * q = [cos(θ/2), n_x*sin(θ/2), n_y*sin(θ/2), n_z*sin(θ/2)]
     *
     * @param angle_rad 旋转角度（弧度）
     * @param axis      旋转轴（单位向量）
     */
    void from_axis_angle(float angle_rad, const Vector3f &axis)
    {
        float half_angle = angle_rad * 0.5f;
        float s = sinf(half_angle);

        q0 = cosf(half_angle);
        q1 = axis.x * s;
        q2 = axis.y * s;
        q3 = axis.z * s;
    }

    /**
     * 四元数归一化
     *
     * 数学公式：
     * q_normalized = q / |q|
     * |q| = sqrt(q0² + q1² + q2² + q3²)
     *
     * 重要性：
     * - 浮点误差会导致四元数不再单位化
     * - 非单位四元数表示错误的旋转角度
     */
    void normalize()
    {
        float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        if (norm > 0) {
            float inv_norm = 1.0f / norm;
            q0 *= inv_norm;
            q1 *= inv_norm;
            q2 *= inv_norm;
            q3 *= inv_norm;
        }
    }

    /**
     * 获取四元数的共轭
     *
     * 数学公式：
     * q* = [q0, -q1, -q2, -q3]
     *
     * 用途：
     * - 计算逆旋转：q⁻¹ = q* / |q|²
     * - 对于单位四元数，q⁻¹ = q*
     */
    Quaternion operator-(void) const
    {
        return Quaternion(q0, -q1, -q2, -q3);
    }

    /**
     * 旋转向量
     *
     * 数学公式：
     * v' = q ⊗ [0, v] ⊗ q*
     *
     * 展开：
     * v' = v + 2q0(q × v) + 2(q × (q × v))
     *
     * @param vec  输入向量（body frame）
     * @return     旋转后的向量（NED frame或相反）
     */
    Vector3f rotate(const Vector3f &vec) const
    {
        // 四元数旋转向量的高效实现
        // 避免临时四元数对象的创建

        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        // 旋转矩阵元素（从四元数构建）
        float r00 = q0q0 + q1q1 - q2q2 - q3q3;
        float r01 = 2.0f * (q1 * q2 - q0 * q3);
        float r02 = 2.0f * (q1 * q3 + q0 * q2);
        float r10 = 2.0f * (q1 * q2 + q0 * q3);
        float r11 = q0q0 - q1q1 + q2q2 - q3q3;
        float r12 = 2.0f * (q2 * q3 - q0 * q1);
        float r20 = 2.0f * (q1 * q3 - q0 * q2);
        float r21 = 2.0f * (q2 * q3 + q0 * q1);
        float r22 = q0q0 - q1q1 - q2q2 + q3q3;

        // 矩阵-向量乘法
        Vector3f result;
        result.x = r00 * vec.x + r01 * vec.y + r02 * vec.z;
        result.y = r10 * vec.x + r11 * vec.y + r12 * vec.z;
        result.z = r20 * vec.x + r21 * vec.y + r22 * vec.z;

        return result;
    }
};
```

---

#### 6.5.3 旋转矩阵（方向余弦矩阵 DCM）

##### 6.5.3.1 DCM数学定义

方向余弦矩阵是一个3×3正交矩阵，描述了两个坐标系之间的旋转关系：

$$R = \begin{bmatrix} \vec{r}_x & \vec{r}_y & \vec{r}_z \end{bmatrix}$$

其中 $\vec{r}_x, \vec{r}_y, \vec{r}_z$ 是行向量（NED坐标系下的基向量）。

**基本旋转矩阵：**

绕X轴旋转（Roll）：
$$R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

绕Y轴旋转（Pitch）：
$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

绕Z轴旋转（Yaw）：
$$R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**复合旋转（Z-Y-X顺序）：**
$$R = R_x(\phi) R_y(\theta) R_z(\psi)$$

##### 6.5.3.2 ArduPilot旋转矩阵代码

**文件位置**: `libraries/AP_Math/matrix3.h`

```cpp
/**
 * 3×3旋转矩阵类
 *
 * 用于表示坐标系变换和向量旋转
 *
 * 数学基础：
 * R = [r11 r12 r13] = [ex_x ey_x ez_x]
 *     [r21 r22 r23]   [ex_y ey_y ez_y]
 *     [r31 r32 r33]   [ex_z ey_z ez_z]
 *
 * 其中 ex, ey, ez 是机体坐标系基向量在NED坐标系的表示
 */

#pragma once

#include <AP_Math/AP_Math.h>

class Quaternion;

class Matrix3f {
public:
    float a.x, a.y, a.z;    // 第一行
    float b.x, b.y, b.z;    // 第二行
    float c.x, c.y, c.z;    // 第三行

    /**
     * 从欧拉角构造旋转矩阵
     *
     * 公式：R = R_x(roll) × R_y(pitch) × R_z(yaw)
     *
     * 完整展开：
     * [ cosψcosθ             -sinψcosφ+cosψsinθsinφ     sinψsinφ+cosψsinθcosφ ]
     * [ sinψcosθ              cosψcosφ+sinψsinθsinφ    -cosψsinφ+sinψsinθcosφ ]
     * [ -sinθ                 cosθsinφ                   cosθcosφ              ]
     *
     * @param roll_rad  横滚角（弧度）
     * @param pitch_rad 俯仰角（弧度）
     * @param yaw_rad   偏航角（弧度）
     */
    void from_euler(float roll_rad, float pitch_rad, float yaw_rad)
    {
        float cr = cosf(roll_rad);
        float cp = cosf(pitch_rad);
        float cy = cosf(yaw_rad);
        float sr = sinf(roll_rad);
        float sp = sinf(pitch_rad);
        float sy = sinf(yaw_rad);

        // 第一行：X轴方向余弦
        a.x = cy * cp;
        a.y = sy * cp;
        a.z = -sp;

        // 第二行：Y轴方向余弦
        b.x = cy * sp * sr - sy * cr;
        b.y = sy * sp * sr + cy * cr;
        b.z = cp * sr;

        // 第三行：Z轴方向余弦
        c.x = cy * sp * cr + sy * sr;
        c.y = sy * sp * cr - cy * sr;
        c.z = cp * cr;
    }

    /**
     * 将旋转矩阵转换为欧拉角
     *
     * 公式：
     * roll  = atan2(r32, r33)
     * pitch = asin(-r31)
     * yaw   = atan2(r21, r11)
     */
    Vector3f to_euler() const
    {
        Vector3f euler;

        // Pitch from -sin(r31)
        // 数值稳定性：使用asin而非直接取反三角函数
        euler.y = asinf(-c.z);

        // Roll from r32/r33
        euler.x = atan2f(b.z, a.z);

        // Yaw from r21/r11
        euler.y = atan2f(b.x, a.x);

        return euler;
    }

    /**
     * 向量变换（旋转）
     *
     * 将向量从一个坐标系变换到另一个坐标系
     *
     * 数学公式：
     * v_ned = R × v_body
     *
     * 或反向变换：
     * v_body = R^T × v_ned
     *
     * @param vec  输入向量
     * @return     变换后的向量
     */
    Vector3f operator*(const Vector3f &vec) const
    {
        Vector3f result;

        // 矩阵乘法：R × v
        result.x = a.x * vec.x + a.y * vec.y + a.z * vec.z;
        result.y = b.x * vec.x + b.y * vec.y + b.z * vec.z;
        result.z = c.x * vec.x + c.y * vec.y + c.z * vec.z;

        return result;
    }

    /**
     * 矩阵乘法：R1 × R2
     *
     * 复合旋转：
     * R_total = R1 × R2
     * 表示先应用R2，再应用R1
     */
    Matrix3f operator*(const Matrix3f &m) const
    {
        Matrix3f result;

        // 行×列乘法
        result.a.x = a.x * m.a.x + a.y * m.b.x + a.z * m.c.x;
        result.a.y = a.x * m.a.y + a.y * m.b.y + a.z * m.c.y;
        result.a.z = a.x * m.a.z + a.y * m.b.z + a.z * m.c.z;

        result.b.x = b.x * m.a.x + b.y * m.b.x + b.z * m.c.x;
        result.b.y = b.x * m.a.y + b.y * m.b.y + b.z * m.c.y;
        result.b.z = b.x * m.a.z + b.y * m.b.z + b.z * m.c.z;

        result.c.x = c.x * m.a.x + c.y * m.b.x + c.z * m.c.x;
        result.c.y = c.x * m.a.y + c.y * m.b.y + c.z * m.c.y;
        result.c.z = c.x * m.a.z + c.y * m.b.z + c.z * m.c.z;

        return result;
    }

    /**
     * 矩阵转置
     *
     * 数学公式：
     * R^T = R⁻¹ (旋转矩阵是正交矩阵)
     *
     * 用于反向变换
     */
    Matrix3f transposed() const
    {
        Matrix3f result;

        // 转置：行列互换
        result.a.x = a.x; result.a.y = b.x; result.a.z = c.x;
        result.b.x = a.y; result.b.y = b.y; result.b.z = c.y;
        result.c.x = a.z; result.c.y = b.z; result.c.z = c.z;

        return result;
    }

    /**
     * 矩阵归一化
     *
     * 浮点误差会导致矩阵列向量不再正交
     * 需要定期归一化以保持数值精度
     *
     * Gram-Schmidt正交化：
     * r_x' = r_x
     * r_y' = r_y - (r_x'·r_y)r_x'
     * r_z' = r_x' × r_y'
     */
    void normalize()
    {
        Vector3f x(a.x, a.y, a.z);
        Vector3f y(b.x, b.y, b.z);
        Vector3f z(c.x, c.y, c.z);

        // 第一列归一化
        float x_norm = x.length();
        if (x_norm > 0) {
            x /= x_norm;
        }

        // 第二列正交化（减去投影）
        float dot = x * y;
        y = y - x * dot;

        // 第二列归一化
        float y_norm = y.length();
        if (y_norm > 0) {
            y /= y_norm;
        }

        // 第三列叉乘（保证右手坐标系）
        z = x % y;

        // 写回矩阵
        a.x = x.x; a.y = x.y; a.z = x.z;
        b.x = y.x; b.y = y.y; b.z = y.z;
        c.x = z.x; c.y = z.y; c.z = z.z;
    }
};
```

---

#### 6.5.4 DCM姿态估计算法

##### 6.5.4.1 DCM算法原理

DCM（方向余弦矩阵）算法通过融合陀螺仪和加速度计/磁力计数据来估计飞行器姿态。

**核心方程：**

$$\dot{R} = R \times \Omega$$

其中：
- $R$：方向余弦矩阵
- $\Omega$：角速度矩阵（反对称矩阵）

**角速度矩阵：**
$$\Omega = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}$$

**离散更新：**
$$R_{k+1} = R_k + \dot{R}_k \times \Delta t$$
$$R_{k+1} = R_k \times (I + \Omega \times \Delta t)$$

##### 6.5.4.2 DCM漂移校正

陀螺仪积分会产生漂移，需要使用加速度计（检测重力）和磁力计（检测北向）进行校正。

**加速度计校正：**
```cpp
/**
 * 误差计算：期望重力向量与测量的夹角
 *
 * 数学公式：
 * error = desired_g × measured_g
 *
 * 其中：
 * - desired_g = [0, 0, 1] (NED坐标系中重力向下)
 * - measured_g = R × body_accel (变换到NED坐标系)
 */
Vector3f error = desired_g % measured_g_normed;

/**
 * 误差修正：
 * - 将误差向量投影到各轴
 * - 漂移校正仅影响绕水平轴的旋转
 * - yaw轴漂移需要磁力计校正
 */

// 忽略z轴误差（加速度计无法检测yaw漂移）
error.z = 0;

// 误差积分
drift_correction += error * KI * dt;

// 应用校正到陀螺仪
omega += drift_correction;
```

##### 6.5.4.3 ArduPilot DCM实现源码

**文件位置**: `libraries/AP_AHRS/AP_AHRS_DCM.cpp`

```cpp
/**
 * DCM姿态估计算法实现
 *
 * 算法流程：
 * 1. 陀螺仪积分更新DCM矩阵
 * 2. 矩阵正交化（Gram-Schmidt）
 * 3. 误差校正（加速度计 + 磁力计）
 * 4. 输出姿态估计（欧拉角/四元数/DCM）
 */

#include <AP_AHRS/AP_AHRS_DCM.h>
#include <AP_Math/AP_Math.h>

// 增益参数
const float Kp = 1.0f;           // 比例校正增益
const float Ki = 0.001f;         // 积分校正增益（用于长期漂移校正）
const float omega_mag_limit = 0.5f; // 角速度限制

// 全局变量
Matrix3f dcm_matrix;              // DCM矩阵
Vector3f gyro_offset;             // 陀螺仪零偏估计
Vector3f drift_correction;        // 漂移校正积分

/**
 * DCM更新 - 每周期调用
 *
 * @param gyro          陀螺仪测量值（弧度/秒）
 * @param accel          加速度计测量值（m/s²）
 * @param mag           磁力计测量值（可选）
 * @param dt            时间步长（秒）
 */
void AP_AHRS_DCM::update(float gyro[3], float accel[3], float mag[3], float dt)
{
    Vector3f gyro_vec(gyro[0], gyro[1], gyro[2]);
    Vector3f accel_vec(accel[0], accel[1], accel[2]);

    // ========== 1. 陀螺仪积分 ==========
    /**
     * DCM矩阵更新公式：
     * dR/dt = R × Ω
     * Ω = [0    -wz  wy]
     *     [ wz   0  -wx]
     *     [-wy  wx   0 ]
     *
     * 离散化（一阶近似）：
     * R_new = R_old + dR × dt
     */
    float wx = gyro_vec.x;
    float wy = gyro_vec.y;
    float wz = gyro_vec.z;

    // 构建反对称矩阵（略去）

    // 矩阵更新
    Matrix3f delta_rotation;
    delta_rotation.a.x = 0;        delta_rotation.a.y = -wz * dt;  delta_rotation.a.z = wy * dt;
    delta_rotation.b.x = wz * dt;  delta_rotation.b.y = 0;         delta_rotation.b.z = -wx * dt;
    delta_rotation.c.x = -wy * dt; delta_rotation.c.y = wx * dt;   delta_rotation.c.z = 0;

    dcm_matrix = dcm_matrix * (Matrix3f::identity() + delta_rotation);

    // ========== 2. 漂移校正 ==========
    /**
     * 校正误差来源：
     * - 加速度计：检测重力方向，校正roll/pitch
     * - 磁力计：检测北向，校正yaw
     *
     * 误差计算方法：叉积误差法
     * error = desired × measured
     *
     * 期望重力向量（NED坐标系）：[0, 0, 1]
     * 期望磁力向量（NED坐标系）：[1, 0, 磁偏角]
     */

    // 加速度计校正
    Vector3f accel_earth = dcm_matrix * accel_vec;  // 变换到NED坐标系
    Vector3f desired_g(0, 0, 1);                   // 期望重力方向

    // 归一化测量值
    float accel_mag = accel_earth.length();
    if (accel_mag > 0) {
        accel_earth /= accel_mag;

        // 计算误差向量（叉积）
        Vector3f error = desired_g % accel_earth;

        /**
         * 校正原理：
         * - 误差方向 = 校正方向
         * - 误差大小 = 角度误差
         * - 应用校正到陀螺仪积分
         */

        // 忽略z轴（加速度计无法检测yaw）
        error.z = 0;

        // 积分校正（消除长期漂移）
        drift_correction += error * Ki * dt;

        // 应用校正到DCM
        dcm_matrix.a += error * Kp;
        dcm_matrix.b += error * Kp;
        dcm_matrix.c += error * Kp;
    }

    // 磁力计校正（如果可用）
    if (mag != nullptr) {
        Vector3f mag_vec(mag[0], mag[1], mag[2]);

        // 归一化磁力计
        float mag_mag = mag_vec.length();
        if (mag_mag > 0) {
            mag_vec /= mag_mag;

            // 去除pitch影响（磁力计水平投影）
            // 变换到水平面
            Vector3f mag_horizontal = dcm_matrix * mag_vec;
            mag_horizontal.z = 0;
            mag_horizontal.normalize();

            // 期望北向
            Vector3f desired_north(1, 0, 0);

            // 计算yaw误差
            float mag_error = desired_north % mag_horizontal;

            // 应用yaw校正
            dcm_matrix.a += mag_error * Kp;
            dcm_matrix.b += mag_error * Kp;
            dcm_matrix.c += mag_error * Kp;
        }
    }

    // ========== 3. 矩阵正交化 ==========
    /**
     * 由于浮点误差，DCM矩阵会逐渐失去正交性
     * 需要定期正交化
     *
     * Gram-Schmidt正交化：
     * x' = x
     * y' = y - (x'·y)x'
     * z' = x' × y'
     */
    dcm_matrix.normalize();

    // ========== 4. 输出更新 ==========
    // 更新欧拉角
    update_euler_from_dcm();

    // 更新四元数
    update_quaternion_from_dcm();
}
```

---

#### 6.5.5 互补滤波器

##### 6.5.5.1 滤波器原理

互补滤波器利用不同传感器在不同频率段的特性进行融合：

```cpp
/**
 * 一阶互补滤波器
 *
 * 数学公式：
 * y[k] = α * x[k] + (1-α) * y[k-1]
 *
 * 频率响应：
 * - 低频：x[k]被衰减，y[k-1]通过 → 通过低频信号
 * - 高频：x[k]通过，y[k-1]被衰减 → 通过高频信号
 *
 * 截止频率：
 * f_cut = α / (2π * Δt)
 */

float complementary_filter(float measurement, float estimate, float alpha, float dt)
{
    return alpha * measurement + (1 - alpha) * estimate;
}
```

##### 6.5.5.2 低通滤波器

**一阶低通滤波器公式：**
$$y[k] = \alpha \cdot x[k] + (1-\alpha) \cdot y[k-1]$$

其中：
$$\alpha = \frac{\Delta t}{RC + \Delta t}$$

$RC$：时间常数，与截止频率关系为 $f_c = \frac{1}{2\pi RC}$

```cpp
/**
 * 一阶低通滤波器实现
 *
 * 用途：
 * - 平滑加速度计信号
 * - 滤除陀螺仪高频噪声
 * - 平滑微分项
 */
class LowPassFilter {
private:
    float alpha;           // 滤波系数
    float output;          // 上次输出
    bool initialized;     // 初始化标志

public:
    LowPassFilter() : output(0), initialized(false) {}

    /**
     * 设置截止频率
     *
     * @param fc     截止频率（Hz）
     * @param dt     采样周期（秒）
     */
    void set_cutoff_frequency(float fc, float dt)
    {
        float RC = 1.0f / (2.0f * M_PI * fc);
        alpha = dt / (RC + dt);
    }

    /**
     * 滤波计算
     */
    float apply(float input)
    {
        if (!initialized) {
            output = input;
            initialized = true;
        }
        output = alpha * input + (1 - alpha) * output;
        return output;
    }

    void reset()
    {
        initialized = false;
        output = 0;
    }
};
```

---

#### 6.5.6 角速度控制器完整实现

##### 6.5.6.1 姿态控制层级

```
┌────────────────────────────────────────────────────────────────────┐
│                    飞控控制层级架构                                  │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│   [位置指令]                                                        │
│        │                                                           │
│        ▼                                                           │
│   ┌─────────┐      [姿态指令]      ┌─────────┐                     │
│   │ 位置控制 │ ─────────────────►  │ 姿态控制 │                     │
│   │  PID    │                      │  P+FF   │                     │
│   └────┬────┘                      └────┬────┘                     │
│        │                                  │                          │
│        │   [角速度指令]                    │                          │
│        └───────────────────────────────► │                          │
│                                           ▼                          │
│                                    ┌─────────────┐                   │
│                                    │ 角速度控制   │                   │
│                                    │   PID       │                   │
│                                    └──────┬──────┘                   │
│                                           │                          │
│                                           ▼                          │
│                                    [电机PWM指令]                     │
│                                                                    │
├────────────────────────────────────────────────────────────────────┤
│ 控制频率：                                                         │
│ - 位置环：10-50Hz                                                  │
│ - 姿态环：50-100Hz                                                 │
│ - 角速度环：200-500Hz (最高优先级)                                  │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

##### 6.5.6.2 ArduPilot姿态控制代码

```cpp
/**
 * AC_AttitudeControl - 姿态控制器实现
 *
 * 文件位置: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
 *
 * 控制流程：
 * 1. 输入姿态指令 → 计算目标角速度 → PID控制 → 电机混控
 */

#include <AC_AttitudeControl.h>
#include <AP_Math/AP_Math.h>

/**
 * 设置目标姿态（四元数）
 *
 * @param target_attitude 目标姿态四元数
 *
 * 数学处理：
 * 1. 计算当前姿态与目标姿态的四元数差
 * 2. 将四元数差转换为欧拉角误差
 * 3. 应用P控制器计算目标角速度
 */
void AC_AttitudeControl::set_attitude_target_quaternion(
    const Quaternion& target_attitude)
{
    // 获取当前姿态四元数
    Quaternion current_attitude = get_quaternion();

    // 计算姿态误差四元数
    // q_error = q_current⁻¹ ⊗ q_target
    Quaternion q_error = current_attitude.inverse() * target_attitude;

    // 转换为欧拉角误差（小角度近似）
    Vector3f euler_error = q_error.to_euler();

    /**
     * 前馈控制（Feedforward）
     *
     * 目的：预测目标角速度，提前做出响应
     *
     * 对于恒定角速度机动：
     * - P控制器只响应姿态误差
     * - 前馈直接提供所需的角速度
     * - 减小跟踪误差
     */

    // 计算目标角速度（度/秒）
    _angvel_target.x = euler_error.x * _p_angle_gain.x + get_roll_rate_ef();
    _angvel_target.y = euler_error.y * _p_angle_gain.y + get_pitch_rate_ef();
    _angvel_target.z = euler_error.z * _p_angle_gain.z + get_yaw_rate_ef();
}

/**
 * 执行角速度控制
 *
 * @description
 * 这是rate环的核心函数，在每帧fast_loop中调用
 *
 * 数学流程：
 * 1. 获取当前角速度测量值
 * 2. 计算角速度误差
 * 3. 应用PID控制器
 * 4. 输出到电机混控
 */
void AC_AttitudeControl::rate_controller_run()
{
    // 获取当前角速度（从陀螺仪）
    Vector3f rate_meas = _ahrs.get_gyro();

    // ========== Roll角速度环 ==========
    float error_r = _angvel_target.x - rate_meas.x;
    float output_r = _pid_rate_roll.get_pid_angvel(error_r, rate_meas.x);

    // ========== Pitch角速度环 ==========
    float error_p = _angvel_target.y - rate_meas.y;
    float output_p = _pid_rate_pitch.get_pid_angvel(error_p, rate_meas.y);

    // ========== Yaw角速度环 ==========
    float error_y = _angvel_target.z - rate_meas.z;
    float output_y = _pid_rate_yaw.get_pid_angvel(error_y, rate_meas.z);

    // ========== 电机混控 ==========
    /**
     * 混控公式（四旋翼X模式）：
     *
     * motor_FL (前左)  = throttle + roll + pitch - yaw  (CW)
     * motor_FR (前右)  = throttle - roll + pitch + yaw   (CCW)
     * motor_BL (后左)  = throttle - roll - pitch - yaw   (CCW)
     * motor_BR (后右)  = throttle + roll - pitch + yaw   (CW)
     *
     * 注意符号约定：
     * - Roll+：向右滚转（右侧电机加速）
     * - Pitch+：向前俯仰（后部电机加速）
     * - Yaw+：顺时针旋转（CCW电机加速产生反扭矩）
     */

    // 各通道范围限制
    output_r = constrain_float(output_r, -1000, 1000);
    output_p = constrain_float(output_p, -1000, 1000);
    output_y = constrain_float(output_y, -1000, 1000);

    // 电机混控
    _motors.output = _hover_throttle;

    _motors.motor_out[0] = _hover_throttle + output_r + output_p - output_y; // FL
    _motors.motor_out[1] = _hover_throttle - output_r + output_p + output_y; // FR
    _motors.motor_out[2] = _hover_throttle - output_r - output_p - output_y; // BL
    _motors.motor_out[3] = _hover_throttle + output_r - output_p + output_y; // BR

    // PWM输出
    _motors.output_pwm();
}
```

---

#### 6.5.7 输入整形算法

##### 6.5.7.1 原因与应用

**输入整形的目的：**
- 抑制系统振荡
- 减少残余振动
- 改善跟踪精度

**数学原理：**

对于二阶系统：
$$\ddot{x} + 2\zeta\omega_n\dot{x} + \omega_n^2x = K \cdot u$$

输入整形器由一系列脉冲组成：
$$h(t) = \sum_{i=1}^{n} A_i \cdot \delta(t - t_i)$$

**ZV/ZVD整形器：**
- ZV (Zero Vibration)：消除指定模态振动
- ZVD (Zero Vibration Derivative)：对参数变化更鲁棒

```cpp
/**
 * 输入整形器实现
 *
 * ZV整形器参数：
 * A = [1, exp(-ζπ/sqrt(1-ζ²))]
 * t = [0, π/ω_d]
 *
 * 其中阻尼比 ζ = cos(φ)，ω_d = ω_n·sin(φ)
 */
class InputShaper {
private:
    float amplitude;        // 整形器增益
    float period;          // 振荡周期
    float damping;         // 阻尼比
    float last_impulse_time; // 上次脉冲时间
    float impulse_sum;     // 脉冲累积

public:
    InputShaper() : last_impulse_time(0), impulse_sum(0) {}

    /**
     * 设置整形器参数
     *
     * @param zeta     阻尼比
     * @param wn       自然频率（rad/s）
     */
    void set_parameters(float zeta, float wn)
    {
        damping = zeta;
        period = 2 * M_PI / (wn * sqrt(1 - zeta * zeta));

        // ZV整形器：两个脉冲
        amplitude = 1.0f / (1 + exp(-zeta * M_PI / sqrt(1 - zeta * zeta)));
    }

    /**
     * 应用输入整形
     *
     * @param input     原始输入指令
     * @param dt        时间步长
     * @return          整形后的输入
     */
    float apply(float input, float dt)
    {
        float output = 0;

        // 检测输入变化（脉冲）
        if (fabs(input - last_input) > threshold) {
            // 生成ZV整形脉冲
            impulse_sum = input * amplitude;
            output = impulse_sum;
            last_impulse_time = 0;
        } else {
            // 累积衰减振荡
            if (impulse_sum > 0.001f) {
                float decay = exp(-damping * last_impulse_time / period);
                impulse_sum = impulse_sum * decay;
                output = impulse_sum;
            }
        }

        last_impulse_time += dt;
        last_input = input;

        return output;
    }
};
```

---

#### 6.5.8 总结：飞控算法数学框架

```
┌─────────────────────────────────────────────────────────────────────┐
│                    飞控算法数学框架总结                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  【传感器层】                                                        │
│  ┌───────────┐    ┌───────────┐    ┌───────────┐                   │
│  │ 陀螺仪    │    │ 加速度计  │    │ 磁力计    │                   │
│  │ 角速度    │    │ 比力      │    │ 磁场      │                   │
│  └─────┬─────┘    └─────┬─────┘    └─────┬─────┘                   │
│        │                │                │                          │
│        └────────────────┼────────────────┘                          │
│                         ▼                                          │
│  【姿态估计层】                                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                                                             │   │
│  │   DCM更新：                                                 │   │
│  │   R_new = R_old × (I + Ω×dt)                               │   │
│  │                                                             │   │
│  │   误差校正：                                                │   │
│  │   error = desired × measured                               │   │
│  │                                                             │   │
│  │   四元数更新：                                              │   │
│  │   q_new = q_old + 0.5 × ω × q_old × dt                    │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                         │                                          │
│                         ▼                                          │
│  【控制层】                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐               │
│  │ 位置环 PID   │  │ 姿态环 P+FF  │  │ 角速度环 PID │               │
│  │              │  │              │  │              │               │
│  │ u = Kp·e     │  │ ω = Kp·e     │  │ u = Kp·e     │               │
│  │    + Ki∫e   │  │    + ω_ff    │  │    + Ki∫e    │               │
│  │    + Kd·de  │  │              │  │    + Kd·dm   │               │
│  └─────────────┘  └─────────────┘  └─────────────┘               │
│                         │                                          │
│                         ▼                                          │
│  【执行层】                                                          │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 电机混控：                                                    │   │
│  │ M_i = throttle ± roll ± pitch ± yaw                         │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

数学符号对照表：
┌─────────────────────────────────────────────────────────────────────┐
│ 符号   │ 含义                          │ 单位                       │
├─────────────────────────────────────────────────────────────────────┤
│ q      │ 四元数                        │ 无量纲                     │
│ R      │ 方向余弦矩阵                  │ 无量纲                     │
│ ω      │ 角速度矢量                    │ rad/s                     │
│ e      │ 误差                          │ rad, m, m/s              │
│ Kp     │ 比例增益                       │ 依控制量而定              │
│ Ki     │ 积分增益                       │ 依控制量而定              │
│ Kd     │ 微分增益                       │ 依控制量而定              │
│ ζ      │ 阻尼比                        │ 无量纲                    │
│ ω_n    │ 自然频率                      │ rad/s                    │
│ dt     │ 时间步长                      │ s                         │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 7. 参数系统

### 7.1 AP_Param - 参数管理框架

**文件位置**: `libraries/AP_Param/AP_Param.h`

参数系统是ArduPilot配置的**核心**，支持EEPROM/Flash存储参数。

#### 7.1.1 参数类型枚举

```cpp
enum ap_var_type {
    AP_PARAM_NONE    = 0,     // 无效类型
    AP_PARAM_INT8    = 1,     // 8位整数 (-128 ~ 127)
    AP_PARAM_INT16   = 2,     // 16位整数 (-32768 ~ 32767)
    AP_PARAM_INT32   = 3,     // 32位整数
    AP_PARAM_FLOAT   = 4,     // 单精度浮点数
    AP_PARAM_VECTOR3F= 5,     // 3维向量（3个float）
    AP_PARAM_GROUP   = 6,     // 参数组（容器）
};
```

#### 7.1.2 参数类模板定义

```cpp
// 参数类模板定义（在AP_Param.h中）
typedef AP_ParamT<int8_t, AP_PARAM_INT8>    AP_Int8;
typedef AP_ParamT<int16_t, AP_PARAM_INT16>  AP_Int16;
typedef AP_ParamT<int32_t, AP_PARAM_INT32>  AP_Int32;
typedef AP_ParamT<float, AP_PARAM_FLOAT>    AP_Float;
typedef AP_ParamV<Vector3f, AP_PARAM_VECTOR3F> AP_Vector3f;
```

#### 7.1.3 创建自定义参数组

```cpp
/**
 * 创建自定义参数组的步骤：
 * 1. 在类中声明参数成员
 * 2. 定义var_info表描述参数结构
 * 3. 在飞控初始化中注册
 */

// ========== 步骤1：声明参数成员 ==========

class MyModule {
public:
    // 整数参数
    AP_Int8  my_mode;           // 模式选择
    AP_Int16 my_count;          // 计数器
    AP_Int16 my_delay;          // 延时参数

    // 浮点参数
    AP_Float my_speed;          // 速度参数
    AP_Float my_gain;           // 增益参数

    // 向量参数
    AP_Vector3f my_offset;     // 偏移量

    /**
     * 参数组信息表
     * 这是一个静态常量数组，告诉参数系统有哪些参数
     */
    static const struct AP_Param::GroupInfo var_info[];
};

// ========== 步骤2：定义var_info表 ==========

const struct AP_Param::GroupInfo MyModule::var_info[] = {
    /**
     * AP_GROUPINFO宏说明：
     * - name: 参数名称（不含前缀，如"MODE"）
     * - idx:  参数索引（同一组内唯一，0-63）
     * - clazz: 所属类名
     * - element: 类中的成员变量名
     * - def:  默认值
     *
     * 最终参数名 = MyModule + "_" + name = "MYMODULE_MODE"
     */
    AP_GROUPINFO("MODE",    0, MyModule, my_mode,    0),   // 默认模式0
    AP_GROUPINFO("COUNT",   1, MyModule, my_count,   10),  // 默认10
    AP_GROUPINFO("DELAY",   2, MyModule, my_delay,   100), // 默认100ms
    AP_GROUPINFO("SPEED",   3, MyModule, my_speed,   1.5f),// 默认1.5
    AP_GROUPINFO("GAIN",    4, MyModule, my_gain,    0.5f),// 默认0.5
    AP_GROUPINFO("OFFSET",  5, MyModule, my_offset,   0.0f),// 默认(0,0,0)

    // 结束标记（必须！）
    AP_GROUPEND
};

// ========== 步骤3：使用参数 ==========

void MyModule::update() {
    // 读取参数值
    int8_t mode = my_mode.get();        // 获取整数参数
    float speed = my_speed.get();        // 获取浮点参数
    Vector3f offset = my_offset.get();  // 获取向量参数

    // 设置参数值
    my_speed.set(2.0f);                 // 设置新值

    // 使用参数
    if (mode == 0) {
        // 模式0的处理
    }
}
```

#### 7.1.4 参数访问方法

```cpp
// ========== 1. 直接访问 ==========

void direct_access_example(MyModule &module) {
    // 读取
    int8_t value = module.my_mode.get();
    float speed = module.my_speed.get();

    // 写入
    module.my_speed.set(2.5f);

    // 保存到EEPROM
    module.my_speed.save();
}

// ========== 2. 按名称查找 ==========

void find_by_name_example() {
    // 通过名称查找参数
    AP_Float *speed_param = (AP_Float *)AP_Param::find("MYMODULE_SPEED");

    if (speed_param != nullptr) {
        float speed = speed_param->get();
        speed_param->set(3.0f);
        speed_param->save();
    } else {
        // 参数不存在
    }
}

// ========== 3. 保存所有参数 ==========

void save_all_params() {
    // 异步保存所有参数
    AP_Param::flush();

    // 同步保存（阻塞）
    AP_Param::flush();
}

// ========== 4. 重置为默认值 ==========

void reset_to_default() {
    module.my_speed.set_default(1.5f);
    module.my_speed.save();
}
```

#### 7.1.5 高级参数定义

```cpp
// ========== 1. 子组（嵌套参数组） ==========

class ParentClass {
public:
    AP_Int8  parent_param;
    MyModule child;        // 子模块

    static const AP_Param::GroupInfo var_info[];
};

const AP_Param::GroupInfo ParentClass::var_info[] = {
    AP_GROUPINFO("PARENT", 0, ParentClass, parent_param, 0),

    // 使用SUBGROUPINFO嵌套子模块参数
    // 最终名称: PARENT_CHILD_MODE, PARENT_CHILD_SPEED 等
    AP_SUBGROUPINFO(child, "CHILD", 1, ParentClass, MyModule),

    AP_GROUPEND
};

// ========== 2. 条件编译参数 ==========

// 使用#if控制参数是否存在
#if MY_MODULE_ENABLED == 1
class MyModule {
public:
    AP_Int8  my_param;
    static const AP_Param::GroupInfo var_info[];
};

const AP_Param::GroupInfo MyModule::var_info[] = {
    AP_GROUPINFO("PARAM", 0, MyModule, my_param, 0),
    AP_GROUPEND
};
#endif

// ========== 3. 帧类型参数 ==========

// 只在特定飞行器类型显示的参数
#define AP_PARAM_FRAME_COPTER    (1<<0)    // 多旋翼
#define AP_PARAM_FRAME_PLANE      (1<<2)    // 固定翼
#define AP_PARAM_FRAME_ROVER      (1<<1)    // 无人车

// 定义只在多旋翼显示的参数
AP_GROUPINFO_FRAME("COPTER_ONLY", 0, MyClass, my_param, 0,
                   AP_PARAM_FRAME_COPTER)
```

#### 7.1.6 常见参数配置

```cpp
// 多旋翼常用参数示例：

// 电机极对数（电调设置）
AP_Int8 MOTOR_PWM_TYPE;     // PWM类型: 0=普通, 1=OneShot125, 2=OneShot42

// PID参数
AP_Float ATC_RATE_RP_P;     // 姿态P增益
AP_Float ATC_RATE_RP_I;     // 姿态I增益
AP_Float ATC_RATE_RP_D;     // 姿态D增益
AP_Float ATC_RATE_Y_P;      // 偏航P增益
AP_Float ATC_RATE_Y_I;      // 偏航I增益
AP_Float ATC_RATE_Y_D;      // 偏航D增益

// 滤波器参数
AP_Float INS_GYRO_FILTER;   // 陀螺仪滤波器截止频率 (Hz)
AP_Float INS_ACCEL_FILTER;  // 加速度计滤波器截止频率 (Hz)

// 飞行模式
AP_Int8 FLTMODE1;           // 通道1的飞行模式
AP_Int8 FLTMODE2;           // 通道2的飞行模式
AP_Int8 FLTMODE3;           // 通道3的飞行模式
AP_Int8 FLTMODE4;           // 通道4的飞行模式
AP_Int8 FLTMODE5;           // 通道5的飞行模式
AP_Int8 FLTMODE6;           // 通道6的飞行模式

// RC输入映射
AP_Int8 RCMAP_ROLL;         // 横滚通道映射
AP_Int8 RCMAP_PITCH;        // 俯仰通道映射
AP_Int8 RCMAP_THROTTLE;     // 油门通道映射
AP_Int8 RCMAP_YAW;          // 偏航通道映射
```

### 7.2 参数框架原理

```
参数存储原理：

1. 【定义阶段】
   - 类定义时声明AP_Int8/AP_Float等参数成员
   - 创建var_info表描述参数结构
   - 每个参数被分配唯一的key值

2. 【初始化阶段】
   - AP_Param::setup() 扫描所有var_info表
   - 为每个参数分配唯一的索引(key)
   - 从EEPROM加载保存的值
   - 如果EEPROM中没有，则使用默认值

3. 【使用阶段】
   - 通过参数对象直接访问
   - 通过AP_Param::find()按名称查找
   - MAVLink可以读写参数

4. 【保存阶段】
   - 参数.set_and_save() 保存单个参数
   - AP_Param::flush() 保存所有参数
   - 数据写入EEPROM/Flash

参数命名空间：
- 全局参数: "PARAM_NAME"
- 参数组内: "GROUP_PARAM_NAME"
- 嵌套参数: "PARENT_CHILD_NAME"

实际参数名示例：
- INS_GYRO_FILTER        (INS库的GYRO_FILTER参数)
- SCHED_LOOP_RATE        (SCHED库的LOOP_RATE参数)
- RC1_MIN                (RC通道1的最小值)
- ATC_RATE_RP_P          (姿态控制的roll/pitch P增益)
```

---

## 8. 通信系统

### 8.1 GCS_MAVLink - 地面站通信

**文件位置**: `libraries/GCS_MAVLink/`

MAVLink是无人机通信的**标准协议**，用于飞控与地面站之间的通信。

#### 8.1.1 MAVLink消息类型

```cpp
// 常用MAVLink消息ID
enum {
    MAVLINK_MSG_ID_HEARTBEAT       = 0,      // 心跳包
    MAVLINK_MSG_ID_SYS_STATUS      = 1,      // 系统状态
    MAVLINK_MSG_ID_ATTITUDE        = 30,     // 姿态数据
    MAVLINK_MSG_ID_GPS_RAW_INT     = 24,     // GPS原始数据
    MAVLINK_MSG_ID_RC_CHANNELS     = 65,     // RC通道数据
    MAVLINK_MSG_ID_MISSION_ITEM    = 39,     // 航点数据
    MAVLINK_MSG_ID_PARAM_VALUE     = 22,     // 参数值
    MAVLINK_MSG_ID_COMMAND_ACK     = 77,     // 命令确认
    MAVLINK_MSG_ID_STATUSTEXT      = 253,    // 状态文本
};
```

#### 8.1.2 发送数据到地面站

```cpp
// ========== 1. 发送姿态数据 ==========

void send_attitude_example(GCS_MAVLINK &gcs) {
    // 自动发送（系统周期性发送）
    // 无需手动调用
}

// ========== 2. 发送状态文本 ==========

void send_statustext_example() {
    // 发送普通信息（绿色文字）
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Hello from ArduPilot!");

    // 发送警告（黄色文字）
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Low battery!");

    // 发送错误（红色文字）
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Sensor error!");

    // 发送调试信息（灰色文字，不显示）
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Debug: value=%d", value);
}

/**
 * MAV_SEVERITY 级别：
 * 0 = Emergency      - 紧急（红色闪烁）
 * 1 = Alert         - 警报（红色）
 * 2 = Critical      - 严重（红色）
 * 3 = Error         - 错误（橙色）
 * 4 = Warning       - 警告（黄色）
 * 5 = Notice        - 通知（淡黄）
 * 6 = Informational - 信息（白色）
 * 7 = Debug         - 调试（灰色）
 */
```

### 8.2 AP_SerialManager - 串口管理

**文件位置**: `libraries/AP_SerialManager/`

管理多个串口的配置和协议分配。

#### 8.2.1 串口协议枚举

```cpp
enum SerialProtocol {
    NONE            = 0,
    MAVLINK         = 1,      // MAVLink地面站
    GPS             = 2,      // GPS接收机
    GPS_MOVEBEACON = 3,      // 移动信标
    ADSB            = 5,      // ADS-B接收机
    LIGHTWARE_SF    = 7,      // 光流传感器
    FRSKY_D         = 9,      // FRSky D协议
    FRSKY_SPORT     = 10,     // FRSky S.Port
    HOTT            = 11,     // HoTT遥测
    MSP             = 12,      // MSP (MultiWii Serial Protocol)
    DJI_FPV         = 13,     // DJI FPV
    // ... 更多
};
```

#### 8.2.2 串口配置参数

```
常用串口参数：
- SERIALn_PROTOCOL: 串口协议 (MAVLink=1, GPS=2等)
- SERIALn_BAUD:     波特率 (115200, 57600, 9600等)
- SERIALn_OPTIONS:  串口选项

示例配置（SERIAL0 = USB/调试串口）：
- SERIAL0_PROTOCOL = 1 (MAVLink)
- SERIAL0_BAUD = 115200

示例配置（SERIAL1 = GPS串口）：
- SERIAL1_PROTOCOL = 2 (GPS)
- SERIAL1_BAUD = 38400
```

#### 8.2.3 使用示例

```cpp
// ========== 1. 获取MAVLink串口 ==========

void mavlink_example() {
    AP_SerialManager *serial = AP::serialmanager();

    // 获取MAVLink串口
    AP_HAL::UARTDriver *uart = serial->find_serial(AP_SerialManager::SerialProtocol::MAVLINK);

    if (uart) {
        // 发送数据
        uart->printf("Hello MAVLink!\n");

        // 检查可用数据
        if (uart->available() > 0) {
            uint8_t c = uart->read();
            // 处理接收到的字节
        }
    }
}

// ========== 2. 获取GPS串口 ==========

void gps_serial_example() {
    AP_SerialManager *serial = AP::serialmanager();

    AP_HAL::UARTDriver *gps_uart = serial->find_serial(AP_SerialManager::SerialProtocol::GPS);

    if (gps_uart) {
        // GPS串口用于连接GPS模块
        // ArduPilot自动处理GPS协议解析
    }
}
```

---

## 9. 飞行器类型

### 9.1 ArduCopter - 多旋翼飞控

**文件位置**: `ArduCopter/`

#### 9.1.1 Copter类结构

```cpp
class Copter : public AP_Vehicle {
private:
    // ========== 核心组件 ==========
    AP_MultiCopter aparm;              // 多旋翼参数
    AC_AttitudeControl *attitude_control;   // 姿态控制
    AC_PosControl *pos_control;        // 位置控制
    AC_WPNav *wp_nav;                 // 航点导航
    AC_Loiter *loiter_nav;            // 悬停导航

    // ========== 状态 ==========
    Mode *flightmode;                 // 当前飞行模式
    AP_BattMonitor battery;            // 电池监控

    // ========== RC输入 ==========
    RC_Channel *channel_roll;          // 横滚通道
    RC_Channel *channel_pitch;         // 俯仰通道
    RC_Channel *channel_throttle;      // 油门通道
    RC_Channel *channel_yaw;          // 偏航通道

    // ========== 传感器 ==========
    AP_RangeFinder rangefinder;       // 距离传感器
    AP_OpticalFlow optflow;           // 光流传感器

    // ========== 任务列表 ==========
    static const AP_Scheduler::Task scheduler_tasks[];
};
```

#### 9.1.2 初始化流程

```cpp
void Copter::setup() {
    // 1. HAL初始化
    AP_HAL::HAL::setup();

    // 2. 加载参数
    load_parameters();

    // 3. 初始化传感器
    ins.init(1000);         // IMU，采样率1000Hz
    compass.init();         // 磁力计
    barometer.init();       // 气压计
    gps.init();            // GPS

    // 4. 初始化电机
    motors->init();

    // 5. 初始化RC输入
    init_rc_in();

    // 6. 初始化飞控特定组件
    init_ardupilot();

    // 7. 初始化任务调度
    AP_Scheduler::init(scheduler_tasks, ARRAY_SIZE(scheduler_tasks));
}

void Copter::loop() {
    // 调用调度器循环
    AP_Scheduler::loop();
}

void Copter::fast_loop() {
    // 1. 读取传感器
    read_inertia();
    update_GPS();
    read_rangefinder();

    // 2. 更新AHRS
    update_AHRS();

    // 3. 更新飞行模式
    update_flight_mode();

    // 4. 执行控制
    attitude_control->rate_controller_run();

    // 5. 输出到电机
    motors_output();
}
```

### 9.2 ArduPlane - 固定翼飞控

**文件位置**: `ArduPlane/`

#### 9.2.1 Plane类结构

```cpp
class Plane : public AP_Vehicle {
private:
    // ========== 控制器 ==========
    AP_RollController rollController;     // 横滚控制器
    AP_PitchController pitchController;   // 俯仰控制器
    AP_YawController yawController;       // 偏航控制器
    AP_TECS tecs;                         // 总能量控制
    AP_L1_Control l1_controller;         // L1导航控制器

    // ========== 参数 ==========
    AP_FixedWing aparm;

    // ========== RC输入 ==========
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;
};
```

### 9.3 Rover - 无人车

**文件位置**: `Rover/`

```cpp
class Rover : public AP_Vehicle {
private:
    // ========== 控制器 ==========
    AP_SteerController steer_controller;   // 转向控制器
    AR_PosControl pos_control;            // 位置控制

    // ========== 车辆特定 ==========
    AP_Vehicle::DiffThrottleControl diff_throttle;  // 差速控制
};
```

---

## 10. 学习路径与实践建议

### 10.1 推荐学习顺序

```
第1周：基础架构
├── 1.1 理解项目结构
│   └── 阅读本文档第1-2章
├── 1.2 HAL层概念
│   └── libraries/AP_HAL/ 相关头文件
├── 1.3 任务调度器
│   └── libraries/AP_Scheduler/
├── 1.4 参数系统
│   └── libraries/AP_Param/
└── 1.5 编译和运行SITL仿真
    └── ./Tools/autotest/sim_vehicle.py

第2周：传感器与姿态
├── 2.1 IMU数据读取
│   └── libraries/AP_InertialSensor/
├── 2.2 GPS数据读取
│   └── libraries/AP_GPS/
├── 2.3 AHRS姿态估计
│   └── libraries/AP_AHRS/
├── 2.4 DCM算法
│   └── AP_AHRS_DCM.cpp
└── 2.5 EKF3原理
    └── libraries/AP_NavEKF3/

第3周：控制系统
├── 3.1 PID控制原理
│   └── libraries/PID/
├── 3.2 姿态控制实现
│   └── libraries/AC_AttitudeControl/
├── 3.3 位置控制实现
│   └── AC_PosControl.cpp
├── 3.4 调参实践
│   └── Mission Planner / QGroundControl
└── 3.5 飞行模式
    └── ArduCopter/mode*.cpp

第4周：通信与任务
├── 4.1 MAVLink协议
│   └── libraries/GCS_MAVLink/
├── 4.2 地面站通信
│   └── Mission Planner连接测试
├── 4.3 任务规划
│   └── AP_Mission/
└── 4.4 安全机制
    └── AP_Arming/, failsafe相关
```

### 10.2 SITL仿真环境搭建

```bash
# Linux系统安装依赖
sudo apt-get update
sudo apt-get install python3-pip python3-matplotlib
sudo apt-get install g++ g++-multilib git

# 克隆并初始化子模块
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# 运行多旋翼仿真
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# 常用参数：
# -v ArduCopter    : 多旋翼
# -v ArduPlane     : 固定翼
# -v Rover         : 无人车
# --console        : 显示控制台
# --map            : 显示地图
```

### 10.3 添加新参数实践

```cpp
// 文件: libraries/AP_MyModule/MyModule.h

#pragma once

#include <AP_Param/AP_Param.h>

class AP_MyModule {
public:
    AP_MyModule();

    void update();

    // ========== 参数定义 ==========
    AP_Int8  my_mode;           // 模式参数
    AP_Float my_speed;          // 速度参数
    AP_Float my_gain;           // 增益参数

    // ========== 参数组信息表 ==========
    static const struct AP_Param::GroupInfo var_info[];
};

// 文件: libraries/AP_MyModule/MyModule.cpp

#include "MyModule.h"

AP_MyModule::AP_MyModule() {
    // 参数注册（在构造函数中自动完成）
}

const struct AP_Param::GroupInfo AP_MyModule::var_info[] = {
    AP_GROUPINFO("MODE",   0, AP_MyModule, my_mode,   0),
    AP_GROUPINFO("SPEED",  1, AP_MyModule, my_speed,  1.0f),
    AP_GROUPINFO("GAIN",   2, AP_MyModule, my_gain,   0.5f),
    AP_GROUPEND
};

void AP_MyModule::update() {
    // 使用参数
    if (my_mode.get() == 0) {
        float output = my_speed.get() * my_gain.get();
        // ...
    }
}

// 在飞控中使用
AP_MyModule my_module;

void copter_init() {
    // 无需额外初始化，AP_Param自动扫描var_info表
}

void copter_update() {
    my_module.update();
}
```

### 10.4 添加新传感器驱动模板

```cpp
// 文件: libraries/AP_MySensor/AP_MySensor.h

#pragma once

#include <AP_HAL/AP_HAL.h>

// 前向声明后端
class AP_MySensor_Backend;

class AP_MySensor {
public:
    AP_MySensor();

    // 获取数据
    float get_data() const { return data; }
    bool healthy() const { return _healthy; }

    // 探测和创建后端
    static AP_MySensor *probe();

    // 参数
    static const struct AP_Param::GroupInfo var_info[];

private:
    friend class AP_MySensor_Backend;

    float data;
    bool _healthy;

    // 后端列表
    AP_MySensor_Backend *_backend;
};

// 文件: libraries/AP_MySensor/AP_MySensor.cpp

#include "AP_MySensor.h"
#include "AP_MySensor_Backend.h"

AP_MySensor::AP_MySensor() {
    // 初始化
    AP_Param::setup_object_defaults(this, var_info);
}

const struct AP_Param::GroupInfo AP_MySensor::var_info[] = {
    AP_GROUPINFO("DATA", 0, AP_MySensor, data, 0),
    AP_GROUPEND
};

// 后端实现
AP_MySensor_Backend::AP_MySensor_Backend(AP_MySensor &frontend)
    : _frontend(frontend) {
}

bool AP_MySensor_Backend::init(AP_HAL::OwnPtr<AP_HAL::Device> dev) {
    _dev = std::move(dev);

    // 检测传感器
    if (!_dev->write_register(REG_ID, ID_VALUE)) {
        return false;
    }

    return true;
}

void AP_MySensor_Backend::update() {
    uint8_t value;
    if (_dev->read_register(REG_DATA, value)) {
        _frontend.data = value;
        _frontend._healthy = true;
    } else {
        _frontend._healthy = false;
    }
}
```

### 10.5 调试技巧

```cpp
// ========== 1. 控制台调试输出 ==========

void debug_example() {
    // 使用hal.console输出
    hal.console->printf("Debug: value=%d\n", value);
    hal.console->printf("Debug: float=%.2f\n", float_value);

    // 使用GCS发送状态文本
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Debug: %s", message);
}

// ========== 2. 性能监控 ==========

void perf_example() {
    // 获取循环时间
    float loop_time = AP::scheduler().get_filtered_loop_time();

    // 获取CPU负载
    float load = AP::scheduler().load_average();

    if (load > 0.8f) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "High CPU load: %.1f%%", load * 100);
    }
}

// ========== 3. 日志记录 ==========

void logging_example() {
    // 记录到DataFlash
    AP::logger().Write("MYDBG", "TimeUS,Value", "QF",
                       AP_HAL::micros64(), value);

    // 记录多个值
    AP::logger().Write("MYDBG", "TimeUS,V1,V2,V3", "Qfff",
                       AP_HAL::micros64(), v1, v2, v3);
}

// ========== 4. 条件调试 ==========

void conditional_debug() {
    // 只在特定条件下输出
    static uint32_t last_debug = 0;
    if (AP_HAL::millis() - last_debug > 1000) {  // 每秒一次
        last_debug = AP_HAL::millis();
        // 输出调试信息
    }
}
```

### 10.6 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 姿态振荡 | PID参数P太大 | 降低P增益，增加D增益 |
| 姿态漂移 | IMU校准问题 | 重新校准加速度计/陀螺仪 |
| 无法解锁 | 安全开关/GPS | 检查安全开关，检查GPS定位 |
| GPS不定位 | 天线连接/遮挡 | 检查天线连接，放到开阔地带 |
| 电机不转 | 未解锁/模式 | 解锁飞控，检查飞行模式 |
| 断线 | 遥控器干扰 | 检查频率，排查干扰源 |
| 定位漂移 | EKF问题 | 重置EKF，检查GPS精度 |
| 高度跳变 | 气压计干扰 | 遮挡气压计，使用GPS辅助 |

### 10.7 推荐学习资源

1. **官方文档**: https://ardupilot.org/
2. **开发者Wiki**: https://ardupilot.org/dev/
3. **源码仓库**: https://github.com/ArduPilot/ardupilot
4. **社区论坛**: https://discuss.ardupilot.org/
5. **MAVLink文档**: https://mavlink.io/en/

---

## 结语

ArduPilot是一个高度模块化、设计精良的开源飞控项目。通过本指南的学习，你应该能够：

1. **理解架构**：掌握分层架构设计，理解HAL、Scheduler、AHRS等核心组件
2. **使用传感器**：能够读取和处理IMU、GPS、气压计、磁力计数据
3. **姿态控制**：理解EKF3和DCM姿态估计算法，掌握PID控制器原理
4. **配置参数**：能够添加和配置自定义参数
5. **开发扩展**：能够添加新的传感器驱动和功能模块

建议按照学习路径逐步深入，先理解整体架构，再深入各个模块的细节实现。最重要的是通过实践（仿真和实飞）来巩固所学知识。

**祝学习顺利！**

---

*文档生成时间: 2025年*
*适用于: ArduPilot 飞行控制器*
