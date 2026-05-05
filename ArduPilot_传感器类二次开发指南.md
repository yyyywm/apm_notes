# ArduPilot 二次开发指南 - VTOL与传感器

> 本文档详细讲解ArduPilot项目中VTOL（垂直起降飞行器）开发以及新增传感器驱动的二次开发方法。

---

## 目录

1. [VTOL二次开发](#一vtol-二次开发深入指南)
2. [新增传感器二次开发](#二新增传感器二次开发指南)

---

## 一、VTOL 二次开发深入指南

### 1.1 VTOL架构概述

ArduPilot中的VTOL实现主要在ArduPlane中，通过`QuadPlane`类实现：

```
┌─────────────────────────────────────────────────────────────────────┐
│                         VTOL 飞行器类型                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. QuadPlane (四旋翼 + 固定翼)                                     │
│     - 最常见的VTOL类型                                              │
│     - 4个或更多旋翼 + 固定翼发动机/副翼/方向舵                      │
│     - 参数: Q_ENABLE=1                                             │
│                                                                     │
│  2. Tailsitter (尾座式)                                            │
│     - 垂直起飞后自动过渡到水平飞行                                   │
│     - 参数: Q_TAILSIT_ENABLE=1 或 Q_FRAME_CLASS=10                │
│                                                                     │
│  3. Tiltrotor (倾转旋翼)                                            │
│     - 旋翼可倾转实现垂直/水平飞行转换                                │
│     - 参数: Q_TILT_ENABLE=1                                        │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 VTOL核心代码结构

#### 主要文件

```
ArduPlane/
├── quadplane.cpp      # QuadPlane核心实现（主要开发文件）
├── tailsitter.cpp     # 尾座式VTOL
├── tiltrotor.cpp      # 倾转旋翼
└── quadplane.h        # QuadPlane类定义
```

#### QuadPlane类核心成员

```cpp
// quadplane.h 核心类定义
class QuadPlane {
public:
    // ============ 参数定义 ============
    AP_Int8  enable;                    // 使能开关

    // 位置/速度控制
    AC_PosControl *pos_control;         // 位置控制器
    AC_WPNav *wp_nav;                   // 航点导航

    // 电机控制
    AP_MotorsMulticopter *motors;        // 多旋翼电机

    // 过渡控制
    AP_Float transition_time_ms;         // 过渡时间
    AP_Float transition_pitch_max;      // 过渡期间最大俯仰角

    // ============ 飞行状态 ============
    enum class VTOLMode {
        Mode::None,                      // 非VTOL模式
        Mode::Hover,                     // 悬停模式（多旋翼）
        Mode::Transition,                // 过渡模式
        Mode::FW                          // 固定翼模式
    };

    VTOLMode vtol_mode;                  // 当前VTOL模式

    // ============ 核心方法 ============

    /**
     * 初始化VTOL系统
     */
    bool init();

    /**
     * 更新VTOL控制（在主循环中每帧调用）
     */
    void update();

    /**
     * 开始过渡到固定翼
     */
    bool transition_to_fw();

    /**
     * 开始过渡到多旋翼
     */
    bool transition_to_vtol();

    /**
     * 悬停控制
     */
    void control_hover();

    /**
     * 辅助控制（在固定翼模式下提供多旋翼辅助）
     */
    void assist_control();
};
```

### 1.3 添加自定义VTOL类型

假设您要添加一种**复合式VTOL**（如涵道风扇+固定翼）：

#### 第1步：创建头文件

```cpp
// ======= ArduPlane/myvtol.h =======

#pragma once

#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/**
 * 自定义复合式VTOL控制器
 *
 * 设计说明：
 * - 结合涵道风扇和固定翼
 * - 可实现垂直起降和高速水平飞行
 */
class MyCustomVTOL {
public:
    /**
     * 构造函数
     */
    MyCustomVTOL(QuadPlane &qp);

    /**
     * 初始化
     */
    bool init();

    /**
     * 更新控制（在每帧调用）
     */
    void update();

    /**
     * 过渡到垂直飞行
     */
    bool transition_to_vertical();

    /**
     * 过渡到水平飞行
     */
    bool transition_to_horizontal();

private:
    QuadPlane &quadplane;                // 引用主QuadPlane

    // 自定义参数
    AP_Int8  ducted_fan_count;           // 涵道风扇数量
    AP_Float ducted_thrust_min;          // 最小推力
    AP_Float ducted_thrust_max;          // 最大推力
    AP_Float transition_speed;           // 过渡速度

    // 内部状态
    bool _initialized;
    uint32_t _transition_start_time;

    /**
     * 涵道风扇电机控制
     */
    void control_ducted_fans(float throttle);

    /**
     * 固定翼控制面控制
     */
    void control_surfaces();
};

#endif // HAL_QUADPLANE_ENABLED
```

#### 第2步：实现源文件

```cpp
// ======= ArduPlane/myvtol.cpp =======

#include "myvtol.h"

const AP_Param::GroupInfo MyCustomVTOL::var_info[] = {

    // @Param: DUCT_COUNT
    // @DisplayName: Number of ducted fans
    // @Description: Number of ducted fan motors
    // @Range: 2 8
    // @User: Standard
    AP_GROUPINFO("DUCT_COUNT", 1, MyCustomVTOL, ducted_fan_count, 4),

    // @Param: DUCT_MIN
    // @DisplayName: Minimum ducted fan thrust
    // @Description: Minimum thrust for ducted fans in VTOL mode
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DUCT_MIN", 2, MyCustomVTOL, ducted_thrust_min, 0.1f),

    // @Param: DUCT_MAX
    // @DisplayName: Maximum ducted fan thrust
    // @Description: Maximum thrust for ducted fans in VTOL mode
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DUCT_MAX", 3, MyCustomVTOL, ducted_thrust_max, 0.8f),

    // @Param: TRANS_SPD
    // @DisplayName: Transition speed
    // @Description: Airspeed required for transition to fixed wing flight
    // @Units: m/s
    // @Range: 5 30
    // @User: Standard
    AP_GROUPINFO("TRANS_SPD", 4, MyCustomVTOL, transition_speed, 12.0f),

    AP_GROUPEND
};

MyCustomVTOL::MyCustomVTOL(QuadPlane &qp)
    : quadplane(qp)
    , _initialized(false)
    , ducted_fan_count(4)
    , ducted_thrust_min(0.1f)
    , ducted_thrust_max(0.8f)
    , transition_speed(12.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool MyCustomVTOL::init()
{
    // 验证配置
    if (ducted_fan_count < 2 || ducted_fan_count > 8) {
        gcs().send_text(MAV_SEVERITY_ERROR, "MyVTOL: Invalid duct count");
        return false;
    }

    // 初始化电机配置
    // ...

    _initialized = true;
    gcs().send_text(MAV_SEVERITY_INFO, "MyVTOL: Initialized");
    return true;
}

void MyCustomVTOL::update()
{
    if (!_initialized) {
        return;
    }

    switch (quadplane.vtol_mode) {
        case QuadPlane::VTOLMode::Hover:
            // 垂直飞行模式：使用涵道风扇
            control_ducted_fans(quadplane.motors->get_throttle());
            break;

        case QuadPlane::VTOLMode::Transition:
            // 过渡模式：同时控制风扇和控制面
            control_ducted_fans(quadplane.motors->get_throttle());
            control_surfaces();
            break;

        case QuadPlane::VTOLMode::FW:
            // 固定翼模式：主要使用控制面
            control_surfaces();
            break;

        default:
            break;
    }
}

void MyCustomVTOL::control_ducted_fans(float throttle)
{
    // 限制油门范围
    throttle = constrain_float(throttle, ducted_thrust_min, ducted_thrust_max);

    // 设置电机输出
    // 根据ducted_fan_count配置电机
    for (int i = 0; i < ducted_fan_count; i++) {
        // 设置PWM输出到对应通道
        // hal.rcout->write(i, throttle_pwm);
    }
}

void MyCustomVTOL::control_surfaces()
{
    // 根据姿态误差调整控制面
    // 这是一个简化的示例
    float roll_error = quadplane.quadplane.attitude_control->get_roll_error();
    float pitch_error = quadplane.quadplane.attitude_control->get_pitch_error();

    // 副翼控制
    // servos->output_trim_to_mix(roll_error * 0.01f);

    // 升降舵控制
    // servos->output_trim_to_mix(pitch_error * 0.01f);
}
```

#### 第3步：注册到主程序

```cpp
// ======= Plane.h 中添加 =======
class Plane : public AP_Vehicle {
public:
    // ...

#if HAL_QUADPLANE_ENABLED
    QuadPlane quadplane;

    // 添加自定义VTOL
    MyCustomVTOL *my_custom_vtol;
#endif
};

// ======= Plane.cpp 初始化中添加 =======
void Plane::init_ardupilot()
{
    // ... 其他初始化 ...

#if HAL_QUADPLANE_ENABLED
    // 初始化标准QuadPlane
    quadplane.init();

    // 初始化自定义VTOL
    if (quadplane.enable > 0) {
        my_custom_vtol = new MyCustomVTOL(quadplane);
        if (my_custom_vtol) {
            my_custom_vtol->init();
        }
    }
#endif
}
```

### 1.4 VTOL过渡控制详解

```
┌─────────────────────────────────────────────────────────────────┐
│                        过渡状态机                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌──────────┐    Q_TRIGGER    ┌──────────────┐                │
│   │          │ ──────────────► │              │                │
│   │  HOVER   │                 │ TRANSITION   │                │
│   │ (多旋翼) │◄─────────────── │   (过渡)     │                │
│   │          │   Q_LOITER      │              │                │
│   └──────────┘                 └──────┬───────┘                │
│                                       │                         │
│                                       │ airspeed > TRANS_MS     │
│                                       │ Q_TRANS_TIME complete   │
│                                       ▼                         │
│                              ┌──────────────┐                   │
│                              │              │                   │
│                              │  FIXED_WING  │◄────────┐         │
│                              │  (固定翼)    │         │         │
│                              │              │         │         │
│                              └──────────────┘         │         │
│                                    ▲                │         │
│                                    │                │         │
│                               Q_RTL                VTOL         │
│                            Q_LAND               REQUEST         │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

#### 过渡控制核心代码

```cpp
bool QuadPlane::transition_to_fw()
{
    // ========== 1. 检查前置条件 ==========
    // 检查空速是否足够
    if (ahrs.airspeed() < aparm.airspeed_min) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VTOL: Low airspeed");
        return false;
    }

    // 检查过渡时间配置
    if (transition_time_ms < 1000) {
        gcs().send_text(MAV_SEVERITY_WARNING, "VTOL: Set TRANS_TIME");
        return false;
    }

    // ========== 2. 开始过渡 ==========
    vtol_mode = VTOLMode::Transition;
    _transition_start_time = AP_HAL::millis();

    // 记录开始过渡时的高度和位置
    _transition_lat = ahrs.get_position().lat;
    _transition_lon = ahrs.get_position().lng;
    _transition_alt = ahrs.get_altitude();

    return true;
}

void QuadPlane::update_transition()
{
    uint32_t elapsed = AP_HAL::millis() - _transition_start_time;
    float progress = (float)elapsed / transition_time_ms.get();

    // 限制进度
    progress = constrain_float(progress, 0.0f, 1.0f);

    // ========== 旋翼到固定翼的过渡 ==========
    if (transition_state == TransitionState::TO_FW) {
        // 线性增加固定翼油门
        float fw_throttle = aparm.throttle_max * progress;
        motors->set_throttle_fw(fw_throttle);

        // 线性减少旋翼电机油门
        float mc_throttle = hover_throttle * (1.0f - progress);
        motors->set_throttle_mc(mc_throttle);

        // 根据进度调整俯仰角（机头逐渐抬起）
        float target_pitch = transition_pitch_max * progress;
        attitude_control->set_pitch_target(target_pitch);

        // 过渡完成判断
        if (progress >= 1.0f && ahrs.airspeed() > transition_airspeed) {
            // 完全切换到固定翼模式
            vtol_mode = VTOLMode::FW;
            motors->set_desired_motor_mask(0); // 停止旋翼电机

            gcs().send_text(MAV_SEVERITY_INFO, "VTOL: Transition complete");
        }
    }

    // ========== 固定翼到旋翼的过渡 ==========
    if (transition_state == TransitionState::TO_VTOL) {
        // 逐渐增加旋翼油门
        float mc_throttle = hover_throttle * progress;
        motors->set_throttle_mc(mc_throttle);

        // 逐渐减小固定翼油门
        float fw_throttle = current_fw_throttle * (1.0f - progress);
        motors->set_throttle_fw(fw_throttle);

        // 根据进度调整俯仰角（机头逐渐放下）
        float target_pitch = transition_pitch_max * (1.0f - progress);
        attitude_control->set_pitch_target(target_pitch);

        // 过渡完成判断
        if (progress >= 1.0f) {
            // 完全切换到旋翼模式
            vtol_mode = VTOLMode::Hover;

            gcs().send_text(MAV_SEVERITY_INFO, "VTOL: Transition to hover complete");
        }
    }
}
```

### 1.5 VTOL开发关键参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| Q_ENABLE | 使能VTOL功能 | 1 |
| Q_FRAME_CLASS | 旋翼类型 | 1(Quad) |
| Q_FRAME_TYPE | 旋翼布局 | 1(X模式) |
| Q_TRANS_TIME | 过渡时间(毫秒) | 5000 |
| Q_ASSIST_SPEED | 辅助速度阈值 | 8-15 m/s |
| Q_VEL_XY_MAX | 最大水平速度 | 10 m/s |
| Q_PILOT_SPEED_UP | 最大上升速度 | 2 m/s |
| Q_PILOT_SPEED_DN | 最大下降速度 | 1.5 m/s |
| Q_LAND_SPEED | 降落速度 | 0.5 m/s |
| Q_ANGLE_MAX | 最大倾角 | 3000 (30度) |

---

## 二、新增传感器二次开发指南

### 2.1 传感器驱动架构

ArduPilot的传感器驱动遵循统一的架构模式：

```
┌─────────────────────────────────────────────────────────────────────┐
│                        传感器驱动架构                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌──────────────────┐                                             │
│   │  AP_InertialSensor │ ← 核心管理类                               │
│   │  (上层API)        │   - 传感器初始化                            │
│   └────────┬─────────┘   - 数据融合                                │
│            │             - 健康监测                                │
│            ▼                                                         │
│   ┌─────────────────────────────────────┐                          │
│   │     AP_InertialSensor_Backend       │ ← 后端基类              │
│   │     (抽象基类)                       │   - 通用接口定义         │
│   └────────┬────────────────────────────┘                          │
│            │                                                         │
│            ▼                                                         │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │                    具体传感器驱动                            │   │
│   │                                                            │   │
│   │  AP_InertialSensor_Invensense   (MPU6000/6500/9250/20602) │   │
│   │  AP_InertialSensor_BMI055       (BMI055)                  │   │
│   │  AP_InertialSensor_BMI088       (BMI088)                  │   │
│   │  AP_InertialSensor_LSM9DS1      (LSM9DS1)                 │   │
│   │  ...                                                     │   │
│   └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 添加新IMU传感器驱动

#### 驱动头文件

```cpp
// ======= libraries/AP_InertialSensor/AP_InertialSensor_MYIMU.h =======
#pragma once

#include "AP_InertialSensor_Backend.h"

/**
 * MYIMU 传感器驱动
 *
 * 支持的传感器：
 * - MYIMU-6000: 6轴IMU (陀螺仪+加速度计)
 * - MYIMU-7000: 6轴IMU (更高精度版本)
 *
 * 数据手册：请参考传感器厂商文档
 */

class AP_InertialSensor_MYIMU : public AP_InertialSensor_Backend {
public:
    /**
     * 构造函数
     *
     * @param imu       父类AP_InertialSensor引用
     * @param dev       SPI/I2C设备句柄
     * @param rotation  传感器相对于飞控的安装方向
     */
    AP_InertialSensor_MYIMU(AP_InertialSensor &imu,
                           AP_HAL::Device *dev,
                           enum Rotation rotation);

    /**
     * 检测传感器是否存在
     *
     * @param imu       父类引用
     * @param dev       设备句柄
     * @param rotation  安装方向
     * @return          成功返回驱动对象，失败返回nullptr
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::Device *dev,
                                            enum Rotation rotation);

    /**
     * 初始化传感器
     */
    void start() override;

    /**
     * 更新传感器数据（在后台任务中调用）
     */
    void update() override;

private:
    /**
     * 读取原始传感器数据
     */
    bool _read_sensor_data();

    /**
     * 传感器初始化配置
     */
    bool _sensor_config();

    /**
     * 检查传感器ID
     */
    uint16_t _check_sensor_id();

    // 设备句柄
    AP_HAL::Device *_dev;

    // 安装方向
    enum Rotation _rotation;

    // 传感器ID
    uint16_t _sensor_id;

    // 采样率
    uint16_t _sample_rate;

    // 陀螺仪数据
    Vector3f _gyro;

    // 加速度计数据
    Vector3f _accel;

    // 温度
    float _temp;

    // 缓冲区
    uint8_t _buffer[14];  // 典型IMU数据长度
};
```

#### 驱动实现文件

```cpp
// ======= libraries/AP_InertialSensor/AP_InertialSensor_MYIMU.cpp =======

#include "AP_InertialSensor_MYIMU.h"

#include <utility>

// 传感器ID定义（从数据手册获取）
#define MYIMU_DEVICE_ID 0x68
#define MYIMU_REG_WHOAMI 0x75
#define MYIMU_REG_PWR_MGMT_1 0x6B
#define MYIMU_REG_ACCEL_XOUT_H 0x3B
#define MYIMU_REG_GYRO_XOUT_H 0x43

// 采样率配置
#define MYIMU_SAMPLE_RATE 1000  // Hz

AP_InertialSensor_MYIMU::AP_InertialSensor_MYIMU(AP_InertialSensor &imu,
                                                 AP_HAL::Device *dev,
                                                 enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(dev)
    , _rotation(rotation)
    , _sensor_id(0)
    , _sample_rate(MYIMU_SAMPLE_RATE)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_MYIMU::probe(
    AP_InertialSensor &imu, AP_HAL::Device *dev, enum Rotation rotation)
{
    // 创建驱动实例
    AP_InertialSensor_MYIMU *backend = new AP_InertialSensor_MYIMU(imu, dev, rotation);

    if (!backend) {
        return nullptr;
    }

    // 检查传感器ID
    if (!backend->_check_sensor_id()) {
        delete backend;
        return nullptr;
    }

    // 注册传感器到IMU管理器
    // 参数：实例号, 陀螺仪健康状态, 加速度计健康状态, 采样率
    if (!imu.register_gyro(backend, _sensor_id, _sample_rate) ||
        !imu.register_accel(backend, _sensor_id, _sample_rate)) {
        delete backend;
        return nullptr;
    }

    return backend;
}

uint16_t AP_InertialSensor_MYIMU::_check_sensor_id()
{
    uint8_t whoami = 0;

    // 读取WHOAMI寄存器
    if (!_dev->read_registers(MYIMU_REG_WHOAMI, &whoami, 1)) {
        return 0;
    }

    // 验证传感器ID
    if (whoami != MYIMU_DEVICE_ID) {
        return 0;
    }

    return _sensor_id = (MAV_SENSOR_ORIENTATION_ROTATION_NONE << 24) |
                        (MAV_SENSOR_IMU_MYIMU << 16) |
                        MYIMU_DEVICE_ID;
}

bool AP_InertialSensor_MYIMU::_sensor_config()
{
    // 唤醒传感器
    uint8_t pwr_mgmt_1 = 0;
    if (!_dev->read_registers(MYIMU_REG_PWR_MGMT_1, &pwr_mgmt_1, 1)) {
        return false;
    }
    pwr_mgmt_1 &= ~0x20;  // 清除睡眠位
    if (!_dev->write_register(MYIMU_REG_PWR_MGMT_1, pwr_mgmt_1)) {
        return false;
    }
    hal.scheduler->delay(10);

    // 配置陀螺仪采样率
    // DLPF = 0 (关闭数字低通滤波器以获得最高采样率)
    // Fchoice = 0b11 (使能FCHOICE)
    uint8_t gyro_config = 0x03;  // 配置值根据数据手册
    if (!_dev->write_register(0x1B, gyro_config)) {  // GYRO_CONFIG寄存器
        return false;
    }

    // 配置加速度计
    uint8_t accel_config = 0x08;  // ±4g量程，根据数据手册
    if (!_dev->write_register(0x1C, accel_config)) {  // ACCEL_CONFIG寄存器
        return false;
    }

    // 配置中断（可选）
    // 启用数据就绪中断

    return true;
}

void AP_InertialSensor_MYIMU::start()
{
    // 配置传感器
    if (!_sensor_config()) {
        return;
    }

    // 申请通知机制
    // 通知：当有新数据时会调用回调函数
    _dev->notify_type(AP_HAL::Device::notify_on_data);

    // 设置采样率
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // 注册SPI/I2C读取（可选，使用轮询）
    // 如果使用FIFO，配置FIFO
}

void AP_InertialSensor_MYIMU::update()
{
    // 检查是否有新数据（通过中断或轮询）
    // 这里演示轮询方式

    // 读取加速度计和陀螺仪数据
    if (!_read_sensor_data()) {
        return;
    }

    // 应用安装方向旋转
    Vector3f accel_rotated = _accel;
    Vector3f gyro_rotated = _gyro;
    accel_rotated.rotate(_rotation);
    gyro_rotated.rotate(_rotation);

    // 提交数据到IMU管理器
    // _imu是父类AP_InertialSensor的引用
    _imu.set_accel(_instance, accel_rotated);
    _imu.set_gyro(_instance, gyro_rotated);
    _imu.set_temperature(_instance, _temp);
}

bool AP_InertialSensor_MYIMU::_read_sensor_data()
{
    // 读取14字节：6加速度计 + 2温度 + 6陀螺仪
    if (!_dev->read_registers(MYIMU_REG_ACCEL_XOUT_H, _buffer, 14)) {
        return false;
    }

    // 解析加速度计数据（16位有符号整数，高字节优先）
    int16_t accel_x = (int16_t)((_buffer[0] << 8) | _buffer[1]);
    int16_t accel_y = (int16_t)((_buffer[2] << 8) | _buffer[3]);
    int16_t accel_z = (int16_t)((_buffer[4] << 8) | _buffer[5]);

    // 解析温度（可选）
    int16_t temp_raw = (int16_t)((_buffer[6] << 8) | _buffer[7]);
    _temp = (temp_raw / 340.0f) + 36.53f;  // 根据数据手册的转换公式

    // 解析陀螺仪数据
    int16_t gyro_x = (int16_t)((_buffer[8] << 8) | _buffer[9]);
    int16_t gyro_y = (int16_t)((_buffer[10] << 8) | _buffer[11]);
    int16_t gyro_z = (int16_t)((_buffer[12] << 8) | _buffer[13]);

    // 转换为物理单位
    // 假设±2000°/s量程 -> 16.4 LSB/°/s
    // 假设±4g量程 -> 8192 LSB/g
    const float GYRO_SCALE = radians(2000.0f) / 32768.0f;
    const float ACCEL_SCALE = 4.0f * 9.81f / 32768.0f;

    _gyro.x = gyro_x * GYRO_SCALE;
    _gyro.y = gyro_y * GYRO_SCALE;
    _gyro.z = gyro_z * GYRO_SCALE;

    _accel.x = accel_x * ACCEL_SCALE;
    _accel.y = accel_y * ACCEL_SCALE;
    _accel.z = accel_z * ACCEL_SCALE;

    return true;
}
```

#### 注册新传感器

```cpp
// ======= libraries/AP_InertialSensor/AP_InertialSensor.cpp =======

// 1. 添加头文件包含
#include "AP_InertialSensor_MYIMU.h"

// 2. 在 detect_backends() 函数中添加检测代码
// 找到ADD_BACKEND宏定义的位置

void AP_InertialSensor::detect_backends(void)
{
    // ... 现有代码 ...

    // 添加MYIMU传感器检测
    #if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // SPI方式
    ADD_BACKEND(AP_InertialSensor_MYIMU::probe(*this,
        hal.spi->get_device("myimu"), ROTATION_NONE));
    #endif

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // 仿真模式
    ADD_BACKEND(AP_InertialSensor_MYIMU::probe(*this,
        hal.spi->get_device("myimu_sim"), ROTATION_NONE));
    #endif
}
```

### 2.3 添加距离传感器（如ToF测距仪）

```cpp
// ======= libraries/AP_RangeFinder/AP_RangeFinder_MYTOF.h =======
#pragma once

#include "AP_RangeFinder.h"

/**
 * MYTOF 超声波/激光测距传感器驱动
 */

class AP_RangeFinder_MYTOF : public AP_RangeFinder_Backend {
public:
    /**
     * 构造函数
     */
    AP_RangeFinder_MYTOF(RangeFinder &ranger, uint8_t instance,
                         AP_RangeFinder_Parameters &params);

    /**
     * 检测传感器
     */
    static AP_RangeFinder_Backend *probe(RangeFinder &ranger,
                                         uint8_t instance,
                                         AP_RangeFinder_Parameters &params,
                                         enum Rotation rotation);

    /**
     * 更新距离数据
     */
    void update(void) override;

private:
    /**
     * 读取距离数据
     */
    bool _read_distance();

    // 设备句柄
    AP_HAL::I2C *_i2c;

    // 缓冲区
    uint8_t _buff[4];
};

// ======= libraries/AP_RangeFinder/AP_RangeFinder_MYTOF.cpp =======

// 实现类似的模式...
// 关键是：
// 1. 继承AP_RangeFinder_Backend
// 2. 实现update()方法
// 3. 调用set_distance()报告距离
// 4. 在RangeFinder.cpp中注册

void AP_RangeFinder_MYTOF::update()
{
    if (!_read_distance()) {
        // 设置为不健康状态
        set_status(RangeFinder::Status::NoData);
        return;
    }

    // 转换为米并设置距离
    float distance_m = _distance_raw * 0.01f;  // 根据传感器单位调整
    set_distance(distance_m);
}
```

### 2.4 添加GPS传感器

```cpp
// ======= libraries/AP_GPS/AP_GPS_MYGPS.h =======
#pragma once

#include "AP_GPS.h"

class AP_GPS_MYGPS : public AP_GPS_Backend {
public:
    AP_GPS_MYGPS(AP_GPS &_gps, AP_HAL::UARTDriver *port, uint8_t _instance);

    // 初始化
    void init(AP_GPS::GPS_State &_state) override;

    // 更新（每帧调用）
    bool read() override;

private:
    // 解析NMEA或UBX协议
    bool _parse_ubx();
    bool _parse_nmea();

    // 解析GPS数据
    bool _parse_gps();

    // 缓存
    uint8_t _buffer[256];
    uint16_t _buffer_len;
};
```

### 2.5 传感器驱动开发最佳实践

```
┌─────────────────────────────────────────────────────────────────────┐
│                    传感器驱动开发检查清单                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  □ 1. 硬件通信                                                     │
│     □ 使用正确的通信接口（SPI/I2C/UART）                           │
│     □ 配置正确的时钟频率                                           │
│     □ 处理通信错误                                                 │
│                                                                     │
│  □ 2. 传感器配置                                                   │
│     □ 根据数据手册配置正确的采样率                                  │
│     □ 配置合适的量程/分辨率                                        │
│     □ 处理传感器初始化失败                                        │
│                                                                     │
│  □ 3. 数据处理                                                     │
│     □ 正确应用安装方向旋转                                         │
│     □ 转换为正确的物理单位                                         │
│     □ 处理数据溢出和异常值                                         │
│                                                                     │
│  □ 4. 性能优化                                                     │
│     □ 使用DMA/FIFO减少CPU占用                                      │
│     □ 使用中断/事件驱动代替轮询                                    │
│     □ 必要时使用低通滤波                                           │
│                                                                     │
│  □ 5. 错误处理                                                     │
│     □ 检测传感器超时                                               │
│     □ 实现看门狗（超时自动重启）                                    │
│     □ 报告传感器健康状态                                           │
│                                                                     │
│  □ 6. 测试                                                         │
│     □ 在SITL中测试                                                 │
│     □ 验证数据合理性                                               │
│     □ 测试错误恢复                                                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 三、开发工具与调试

### 3.1 SITL仿真测试

```bash
# 启动带GPS仿真的SITL
./Tools/autotest/sim_vehicle.py --vehicle=ArduPlane --frame=quad

# 在MAVLink控制台测试传感器
# 连接地面站查看传感器状态
```

### 3.2 添加调试输出

```cpp
// 在传感器驱动中添加调试输出
void AP_InertialSensor_MYIMU::debug_print()
{
    hal.console->printf("MYIMU: accel=(%.2f, %.2f, %.2f) gyro=(%.2f, %.2f, %.2f)\n",
        _accel.x, _accel.y, _accel.z,
        _gyro.x, _gyro.y, _gyro.z);
}
```

---

## 四、编译与测试

### 4.1 本地编译

```bash
# 配置编译环境（首次）
./waf configure --board=matekf405-wing

# 编译固定翼（包含VTOL）
./waf build --target=ArduPlane

# 或者只编译特定硬件
./waf build --board=matekf405-wing --target=ArduPlane
```

**常用硬件板型：**

| 板型 | 命令 | 说明 |
|------|------|------|
| Matek F405-WING | `--board=matekf405-wing` | 固定翼 |
| Matek F405 | `--board=matekf405` | 多旋翼 |
| CubeOrange+ | `--board=cubeorange` | Pixhawk |
| Pixhawk 1 | `--board=pixhawk1` | 经典Pixhawk |

### 4.2 仿真测试

```bash
# 启动SITL仿真（多旋翼VTOL）
./Tools/autotest/sim_vehicle.py --vehicle=ArduPlane --frame=quad --map

# 模拟GPS丢失
./Tools/autotest/sim_vehicle.py --vehicle=ArduPlane --frame=quad --gps-lock=0

# 查看帮助
./Tools/autotest/sim_vehicle.py --help
```

---

## 五、参考资料

| 资源 | 链接 | 说明 |
|------|------|------|
| 官方开发文档 | https://ardupilot.org/dev/ | 最权威的开发指南 |
| 代码仓库 | https://github.com/ArduPilot/ardupilot | 最新代码 |
| 社区论坛 | https://discuss.ardupilot.org/ | 问题解答 |
| 开发邮件列表 | https://groups.google.com/forum/#!forum/ardupilot-devel | 开发讨论 |

---

*本文档由ArduPilot学习指南生成，供二次开发参考使用。*
