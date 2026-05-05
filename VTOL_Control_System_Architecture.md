# ArduPilot VTOL 控制系统架构详解

## 概述

本文档详细介绍 ArduPilot 中 VTOL（垂直起降飞行器）的控制系统架构，包括控制器层级、数据流向、物理模型对应关系以及核心代码解析。

---

## 一、VTOL 控制系统概览

### 1.1 为什么需要多层级控制器

VTOL 飞行器需要在三维空间（高度、水平位置、航向）进行精确控制，同时还要处理多旋翼模式和固定翼模式之间的切换。为了实现稳定可靠的飞行控制，ArduPilot 采用了一套成熟的**层级控制架构**：

```
任务目标（航点、姿态命令）
        ↓
    位置控制器（速度规划）
        ↓
    姿态控制器（角度控制）
        ↓
    速率控制器（角速度控制）
        ↓
    电机输出（实际推力）
```

这种设计的核心思想是：**每层控制器只关注一个物理量，通过逐层分解将高层的"我要去哪里"转化为低层的"电机怎么转"**。

### 1.2 三大核心控制器

VTOL 系统主要使用三类控制器：

| 控制器类型 | 类名 | 功能 | 控制物理量 |
|-----------|------|------|-----------|
| **位置控制器** | AC_PosControl | 位置→速度/加速度 | 位置、速度 |
| **姿态控制器** | AC_AttitudeControl_Multi | 角度→角速度 | Roll、Pitch、Yaw |
| **任务控制器** | AC_WPNav | 航点导航 | 航线规划 |

这些控制器在 `QuadPlane` 类中的定义如下（来自 [quadplane.h:238-241](../ArduPlane/quadplane.h#L238-L241)）：

```cpp
AC_AttitudeControl_Multi *attitude_control;  // 姿态控制器
AC_PosControl *pos_control;                  // 位置控制器
AC_WPNav *wp_nav;                            // 航点导航
AC_Loiter *loiter_nav;                       // 盘旋导航
```

---

## 二、位置控制器 (AC_PosControl)

### 2.1 物理模型

位置控制器的目标是：**让飞行器从当前位置移动到目标位置**。

在物理世界中，这通过控制两个方向的运动实现：

| 方向 | NED坐标系 | 控制对象 | 控制输出 |
|------|-----------|---------|---------|
| **水平方向** | N(北)、E(东) | 水平位置/速度 | 飞行器倾角 |
| **垂直方向** | D(下) | 高度/爬升率 | 电机推力 |

**为什么水平用倾角、垂直用推力？**

这是由多旋翼的物理特性决定的：
- **垂直运动**：直接控制电机总推力，推力越大上升越快
- **水平运动**：通过倾斜机身产生水平分力，倾角越大水平加速度越大

### 2.2 水平位置控制器 (NE Controller)

#### 2.2.1 输入

```cpp
// 设置目标速度和加速度
pos_control->input_vel_accel_NE_m(Vector2f vel_ne_ms, Vector2f accel_ne_mss, bool limit_output);
```

典型调用来自 [quadplane.cpp:2915-2930](../ArduPlane/quadplane.cpp#L2915-L2930)：

```cpp
// 起飞时：向上爬升，速度为默认上升速度
if (poscontrol.get_state() == QPOS_TAKEOFF) {
    set_climb_rate_ms(wp_nav->get_default_speed_up_ms());
}

// 降落时：根据高度调整速度
set_climb_rate_ms(-1.0 * position2_target_speed_ms);  // 1m/s下降
```

#### 2.2.2 控制流程

```
目标速度 (vel_ne_ms)
       ↓
   [速度误差计算]
       ↓
   [P/PD 控制器]
       ↓
目标加速度 (accel_ne_mss)
       ↓
[加速度→倾角转换]
       ↓
目标Roll角 + 目标Pitch角
```

关键转换函数（来自 [AC_PosControl.cpp](../libraries/AC_AttitudeControl/AC_PosControl.cpp)）：

```cpp
// 将水平加速度转换为倾角
void AC_PosControl::accel_NE_mss_to_lean_angles_rad(
    float accel_n_mss,    // 北向加速度
    float accel_e_mss,    // 东向加速度
    float& roll_target_rad,   // 输出目标滚转角
    float& pitch_target_rad   // 输出目标俯仰角
) {
    // 基于力平衡：a = g * tan(θ) ≈ g * θ
    // 所以 θ = a / g
    roll_target_rad = atan2f(accel_e_mss, GRAVITY_M_SS);
    pitch_target_rad = -atan2f(accel_n_mss, GRAVITY_M_SS);
}
```

**物理意义**：当飞行器倾斜 θ 角时，推力在水平方向的分力为 `T * sin(θ)`，当 θ 很小时约等于 `T * θ`。如果总推力等于重力 (`T = mg`)，则水平加速度 `a = g * θ`。

#### 2.2.3 关键参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| WPNAV_ACCEL | 水平加速度限制 | 250 cm/s² |
| WPNAV_SPEED | 巡航速度 | 1500 cm/s |
| WPNAV_RADIUS | 航点半径 | 200 cm |

### 2.3 垂直位置控制器 (D Controller)

#### 2.3.1 输入

```cpp
// 设置目标爬升率（向下为正）
pos_control->input_vel_accel_D_m(float vel_d_ms, float accel_d_mss, bool limit_output);
```

典型调用来自 [quadplane.cpp:1137-1141](../ArduPlane/quadplane.cpp#L1137-L1141)：

```cpp
void QuadPlane::set_climb_rate_ms(float target_climb_rate_ms)
{
    // NED坐标系中向下为正，所以取反
    float vel_d_m = -target_climb_rate_ms;
    pos_control->input_vel_accel_D_m(vel_d_m, 0, false);
}
```

#### 2.3.2 控制流程

```
目标爬升率 (cm/s)
       ↓
   [速度误差计算]
       ↓
   [P/PD 控制器]
       ↓
目标加速度 (m/s²)
       ↓
[加速度→推力转换]
       ↓
目标推力 (0-1)
```

**物理模型**：

垂直方向的运动方程：
```
F_net = m * a
T - mg = m * a
T = m * (g + a)
```

其中：
- T 是电机总推力
- m 是飞行器质量
- g 是重力加速度 (9.8 m/s²)
- a 是目标垂直加速度

#### 2.3.3 悬停推力

为了保持悬停，需要的推力等于重力：

```cpp
// 获取悬停油门位置
float thr_mid = motors->get_throttle_hover();  // 约 0.5 左右
```

这个值通常通过以下方式获取：
1. **自动估计**：飞行器起飞后，飞控会根据实际表现自动学习
2. **手动设置**：通过 `Q_M_HOVER_LEARN` 参数配置

### 2.4 位置控制器核心函数

来自 [AC_PosControl.h:185-293](../libraries/AC_AttitudeControl/AC_PosControl.h#L185-L293)：

```cpp
// 水平方向
void NE_set_max_speed_accel_m(float speed_ms, float accel_mss);     // 设置速度/加速度限制
void input_vel_accel_NE_m(Vector2f& vel_ne_ms, Vector2f& accel_ne_mss); // 输入目标速度
void NE_update_controller();   // 运行NE控制器

// 垂直方向
void D_set_max_speed_accel_m(float decent_speed_max_ms, float climb_speed_max_ms, float accel_max_d_mss);
void input_vel_accel_D_m(float& vel_d_ms, float accel_d_mss, bool limit_output);
void D_update_controller();    // 运行D控制器
```

---

## 三、姿态控制器 (AC_AttitudeControl_Multi)

### 3.1 物理模型

姿态控制器的目标是：**让飞行器达到并保持目标姿态角**。

三轴控制的物理原理：

| 轴 | 控制对象 | 物理原理 | 电机控制 |
|----|---------|---------|---------|
| **Roll (滚转)** | 机身左右倾斜 | 左右电机差速产生力矩 | 外侧电机加速 |
| **Pitch (俯仰)** | 机身前后倾斜 | 前后电机差速产生力矩 | 前后电机加速 |
| **Yaw (偏航)** | 机身朝向 | 电机反扭矩差速 | 对角电机减速 |

### 3.2 Roll/Pitch 控制

#### 3.2.1 输入

```cpp
// 输入目标姿态角（欧拉角）
void input_euler_angle_roll_pitch_yaw_cd(
    int32_t roll_cd,     // 目标滚转角（centidegrees）
    int32_t pitch_cd,    // 目标俯仰角
    int32_t yaw_cd,      // 目标偏航角
    bool use_slew         // 是否使用平滑过渡
);

// 或者输入角速度
void input_euler_angle_roll_pitch_euler_rate_yaw_cd(
    int32_t roll_cd,
    int32_t pitch_cd,
    int32_t yaw_cd
);
```

典型调用来自 [quadplane.cpp:2750-2754](../ArduPlane/quadplane.cpp#L2750-L2754)：

```cpp
// 自动模式下的姿态控制
attitude_control->input_euler_angle_roll_pitch_yaw_cd(
    plane.nav_roll_cd,    // 来自航点导航
    plane.nav_pitch_cd,   // 来自航点导航
    wp_nav->get_yaw(),    // 来自航点导航
    true
);
```

#### 3.2.2 控制流程（两层PID）

```
目标姿态角 (Roll/Pitch/Yaw)
       ↓
   [外环：姿态 PID]
       ↓
目标角速度 (Rate)
       ↓
   [内环：速率 PID]
       ↓
电机推力差 → 实际角加速度
```

**Roll/Pitch 控制器代码结构**（来自 [AC_AttitudeControl_Multi.cpp](../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp)）：

```cpp
void AC_AttitudeControl_Multi::rate_controller_run()
{
    // 获取当前角速度（来自IMU）
    Vector3f angular_velocity;
    _ahrs.get_gyro(angular_velocity);

    // 计算误差
    Vector3f rate_error = rate_desired - angular_velocity;

    // Roll PID 控制
    float roll_out = _roll_pid.update_all(rate_error.x, _dt);

    // Pitch PID 控制
    float pitch_out = _pitch_pid.update_all(rate_error.y, _dt);

    // Yaw PID 控制
    float yaw_out = _yaw_pid.update_all(rate_error.z, _dt);

    // 输出到电机（转换为电机推力差）
    _motors->set_roll(roll_out);
    _motors->set_pitch(pitch_out);
    _motors->set_yaw(yaw_out);
}
```

### 3.3 Yaw (偏航) 控制

#### 3.3.1 偏航的物理原理

偏航控制利用的是**电机反扭矩**原理：

```
        电机1(CW)          电机2(CCW)
           ↓                 ↓
         ┌─────────────────────┐
         │                     │
         │      机  身         │
         │                     │
         └─────────────────────┘
           ↺ 反作用扭矩        ↻ 反作用扭矩
```

当顺时针(CW)电机加速时，会产生逆时针的反作用扭矩，使机身顺时针偏转。

#### 3.3.2 偏航控制模式

ArduPilot 支持两种偏航控制模式：

1. **速率模式**：直接控制偏航角速度
2. **角度模式**：控制目标航向角

典型调用来自 [quadplane.cpp:1156](../ArduPlane/quadplane.cpp#L1156)：

```cpp
void QuadPlane::hold_hover(float target_climb_rate_cms)
{
    // ...
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds(false));
    // ...
}

void QuadPlane::multicopter_attitude_rate_update(float yaw_rate_cds)
{
    attitude_control->rate_controller_run();  // 内部会处理偏航
}
```

### 3.4 姿态控制器核心函数

来自 [AC_AttitudeControl_Multi.h](../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h)：

```cpp
// 姿态输入
void input_euler_angle_roll_pitch_yaw_cd(int32_t roll_cd, int32_t pitch_cd, int32_t yaw_cd, bool use_slew);
void input_euler_angle_roll_pitch_euler_rate_yaw_cd(int32_t roll_cd, int32_t pitch_cd, int32_t yaw_cd);

// 角速度输入
void input_rate_roll_pitch_yaw(float roll_rate_rads, float pitch_rate_rads, float yaw_rate_rads);

// 运行速率控制器（核心输出函数）
void rate_controller_run();

// 设置PID参数
void set_roll_rate_pid(const AC_PID& pid);
void set_pitch_rate_pid(const AC_PID& pid);
void set_yaw_rate_pid(const AC_PID& pid);
```

---

## 四、任务控制器 (AC_WPNav)

### 4.1 功能概述

任务控制器负责**航线规划**，将一系列航点转换为具体的位置目标。

### 4.2 核心功能

| 功能 | 类 | 说明 |
|------|-----|------|
| **航点导航** | AC_WPNav | 飞往下一个航点 |
| **盘旋** | AC_Loiter | 在当前位置盘旋 |

### 4.3 航点导航流程

```cpp
// 设置目标航点
wp_nav->set_wp_destination_NED_m(target_ned_m);

// 运行导航计算
wp_nav->update_wpnav();

// 获取结果
plane.nav_roll_cd = wp_nav->get_roll();      // 目标滚转角
plane.nav_pitch_cd = wp_nav->get_pitch();   // 目标俯仰角
```

典型调用来自 [quadplane.cpp:3293-3331](../ArduPlane/quadplane.cpp#L3293-L3331)：

```cpp
void QuadPlane::waypoint_controller(void)
{
    setup_target_position();

    const Location &loc = plane.next_WP_loc;

    // 设置航点目标
    if (!loc.same_loc_as(last_auto_target) || now - last_loiter_ms > 500) {
        wp_nav->set_wp_destination_NED_m(poscontrol.target_ned_m);
        last_auto_target = loc;
    }

    // 运行航点导航
    wp_nav->update_wpnav();

    // 获取目标姿态
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();

    // 倾斜角度用于前进推力分配（针对倾斜旋翼）
    assign_tilt_to_fwd_thr();

    // 姿态控制
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(
        plane.nav_roll_cd,
        plane.nav_pitch_cd,
        wp_nav->get_yaw(),
        true
    );

    // 高度控制
    set_climb_rate_ms(assist_climb_rate_cms() * 0.01);
    run_z_controller();
}
```

---

## 五、控制链路完整解析

### 5.1 数据流向图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           任务层 (AUTO 模式)                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │  起飞任务   │  │  航点任务   │  │  降落任务   │  │  盘旋任务   │     │
│  │MAV_CMD_VTOL│  │   航点1-N   │  │MAV_CMD_VTOL │  │  LOITER     │     │
│  │  _TAKEOFF  │  │             │  │   _LAND     │  │             │     │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘     │
└─────────┼─────────────────┼─────────────────┼─────────────────┼───────────┘
          │                 │                 │                 │
          ↓                 ↓                 ↓                 ↓
┌─────────────────────────────────────────────────────────────────────────────┐
│                      位置控制器 (AC_PosControl)                              │
│  ┌─────────────────────────────────┐  ┌─────────────────────────────────┐ │
│  │      NE 水平位置控制器           │  │       D 垂直位置控制器            │ │
│  │  输入: 目标位置/速度             │  │  输入: 目标高度/爬升率           │ │
│  │  输出: 目标倾角 (Roll/Pitch)     │  │  输出: 目标推力                   │ │
│  │                                 │  │                                  │ │
│  │  速度误差 → PID → 加速度         │  │  速度误差 → PID → 推力          │ │
│  │  加速度 → atan(a/g) → 倾角       │  │  F = m(g+a) → 油门              │ │
│  └──────────────┬──────────────────┘  └───────────────┬────────────────┘ │
└─────────────────┼──────────────────────────────────────┼──────────────────┘
                  │                                       │
                  ↓                                       ↓
┌─────────────────────────────────────────────────────────────────────────────┐
│                  姿态控制器 (AC_AttitudeControl_Multi)                       │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                    双环 PID 控制器                                       │ │
│  │                                                                       │ │
│  │   外环(姿态环): 目标角度 → 目标角速度                                   │ │
│  │     Roll:  roll_error → roll_rate_pid → roll_rate_des               │ │
│  │     Pitch: pitch_error → pitch_rate_pid → pitch_rate_des            │ │
│  │     Yaw:   yaw_error → yaw_rate_pid → yaw_rate_des                  │ │
│  │                                                                       │ │
│  │   内环(速率环): 目标角速度 → 电机推力差                                │ │
│  │     Roll:  (rate_des - rate) → PID → motor_diff_roll                │ │
│  │     Pitch: (rate_des - rate) → PID → motor_diff_pitch               │ │
│  │     Yaw:   (rate_des - rate) → PID → motor_diff_yaw                 │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────┬───────────────────────────────────────┘
                                      │
                                      ↓
┌─────────────────────────────────────────────────────────────────────────────┐
│                         电机输出层 (AP_Motors)                               │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                     多旋翼电机混控                                       │ │
│  │                                                                       │ │
│  │   基础推力: throttle (来自D控制器)                                     │ │
│  │   Roll差速: +motor1 -motor2 +motor3 -motor4                            │ │
│  │   Pitch差速: -motor1 +motor2 -motor3 +motor4                         │ │
│  │   Yaw差速: (取决于电机转向)                                           │ │
│  │                                                                       │ │
│  │   最终输出: motor1 = throttle + roll + pitch + yaw                   │ │
│  │             motor2 = throttle - roll + pitch - yaw                   │ │
│  │             motor3 = throttle + roll - pitch - yaw                   │ │
│  │             motor4 = throttle - roll - pitch + yaw                   │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.2 不同飞行模式的控制差异

| 模式 | 位置目标来源 | 高度控制 | Roll/Pitch控制 | Yaw控制 |
|------|-------------|---------|---------------|--------|
| **QStabilize** | RC摇杆输入 | RC油门直接映射 | RC摇杆→倾角 | RC偏航 |
| **QHover** | 自动保持当前位置 | RC→爬升率→位置PID | 自动水平位置PID | RC偏航 |
| **QLoiter** | GPS位置保持 | 自动高度PID | GPS位置PID | 自动/RC |
| **AUTO** | 航点导航 | 高度误差→爬升率 | 航点→倾角 | 航向保持 |

### 5.3 QStabilize 模式详细解析

来自 [mode_qstabilize.cpp:62-96](../ArduPlane/mode_qstabilize.cpp#L62-L96)：

```cpp
void ModeQStabilize::run()
{
    const uint32_t now = AP_HAL::millis();

    // 尾座式过渡期间使用固定翼控制器
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        Mode::run();
        return;
    }

    // 为推进电机分配倾斜角度（针对倾斜旋翼）
    plane.quadplane.assign_tilt_to_fwd_thr();

    // ESC校准模式
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    // ====== QStabilize 核心控制 ======

    // 1. 获取RC油门输入并缩放
    float pilot_throttle_scaled = quadplane.get_pilot_throttle();

    // 2. 执行姿态稳定 + 油门控制
    quadplane.hold_stabilize(pilot_throttle_scaled);

    // 3. 用固定翼舵面辅助稳定Roll和Pitch
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // 4. 方向舵居中
    output_rudder_and_steering(0.0);
}
```

其中 `hold_stabilize` 函数（[quadplane.cpp:1061-1080](../ArduPlane/quadplane.cpp#L1061-L1080)）：

```cpp
void QuadPlane::hold_stabilize(float throttle_in)
{
    // 姿态控制：调用速率控制器
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds(false));

    if ((throttle_in <= 0) && !air_mode_active()) {
        // 油门为0时，电机怠速
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        relax_attitude_control();
    } else {
        // 正常飞行，油门直接输出
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        attitude_control->set_throttle_out(throttle_in, should_boost, 0);
    }
}
```

### 5.4 QHover 模式详细解析

来自 [mode_qhover.cpp:48-78](../ArduPlane/mode_qhover.cpp#L48-L78)：

```cpp
void ModeQHover::run()
{
    quadplane.assist.check_VTOL_recovery();

    const uint32_t now = AP_HAL::millis();

    // 尾座式过渡期间使用固定翼控制器
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        Mode::run();
        return;
    }

    if (quadplane.throttle_wait) {
        // 等待油门（未解锁）
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->D_relax_controller(0);
    } else {
        // ====== 位置保持 + 高度控制 ======
        plane.quadplane.assign_tilt_to_fwd_thr();

        // 根据爬升率目标保持悬停
        // 输入是：RC摇杆 → 爬升率 → 位置控制器
        quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());
    }

   翼舵面辅助 // 固定稳定
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // 方向舵居中
    output_rudder_and_steering(0.0);

    // 可能的螺旋桨反转保护
    quadplane.assist.output_spin_recovery();
}
```

`hold_hover` 函数（[quadplane.cpp:1146-1162](../ArduPlane/quadplane.cpp#L1146-L1162)）：

```cpp
void QuadPlane::hold_hover(float target_climb_rate_cms)
{
    // 电机全速运行
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 设置垂直速度/加速度限制
    pos_control->D_set_max_speed_accel_m(
        get_pilot_velocity_z_max_dn_m(),  // 最大下降速度
        pilot_speed_z_max_up_ms,           // 最大上升速度
        pilot_accel_z_mss                  // 垂直加速度
    );

    // 姿态控制
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds(false));

    // 位置控制：爬升率 → 目标速度 → PID → 推力
    set_climb_rate_ms(target_climb_rate_cms * 0.01);
    run_z_controller();
}
```

---

## 六、自动飞行时的油门计算

### 6.1 核心逻辑

在没有遥控器输入的自动模式下，油门由**高度误差**决定。

来自 [quadplane.cpp:1479-1500](../ArduPlane/quadplane.cpp#L1479-L1500)：

```cpp
float QuadPlane::assist_climb_rate_cms(void) const
{
    float climb_rate_cms;

    if (plane.control_mode->does_auto_throttle()) {
        // ====== 自动油门模式 ======
        // 核心公式：爬升率 = 高度误差 / 时间常数
        // 目标：在10秒内修正高度误差
        climb_rate_cms = plane.calc_altitude_error_cm() * 0.1f;
    } else {
        // ====== 手动油门模式（FBWA等）======
        // 根据俯仰角和油门输入估算
        climb_rate_cms = plane.g.flybywire_climb_rate *
                         (plane.nav_pitch_cd / (plane.aparm.pitch_limit_max * 100));
        climb_rate_cms *= plane.get_throttle_input();
    }

    // 限制在最大速度范围内
    climb_rate_cms = constrain_float(
        climb_rate_cms,
        -wp_nav->get_default_speed_down_ms() * 100.0,  // 最大下降
        wp_nav->get_default_speed_up_ms() * 100.0       // 最大上升
    );

    // 2秒内平滑过渡（防止突然跳变）
    const uint32_t ramp_up_time_ms = 2000;
    const uint32_t dt_since_start = last_pidz_active_ms - last_pidz_init_ms;
    if (dt_since_start < ramp_up_time_ms) {
        climb_rate_cms = linear_interpolate(
            0, climb_rate_cms,
            dt_since_start, 0, ramp_up_time_ms
        );
    }

    return climb_rate_cms;
}
```

### 6.2 物理意义

**为什么是 `altitude_error * 0.1`？**

这实际上是一个 **P 控制器**，其中：
- `altitude_error_cm` 是高度误差（目标高度 - 当前高度）
- `0.1` 是比例增益
- 物理意义：**在10秒内修正完高度误差**

例如：
- 高度误差 = 1000 cm (10米)
- 目标爬升率 = 1000 * 0.1 = 100 cm/s
- 含义：以1m/s的速度爬升，10秒后到达目标高度

---

## 七、关键参数汇总

### 7.1 位置控制参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| WPNAV_SPEED | 航点巡航速度 | 1500 cm/s |
| WPNAV_SPEED_UP | 最大上升速度 | 250 cm/s |
| WPNAV_SPEED_DN | 最大下降速度 | 150 cm/s |
| WPNAV_ACCEL | 水平加速度 | 250 cm/s² |
| WPNAV_RADIUS | 航点半径 | 200 cm |

### 7.2 姿态控制参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| ATC_ANG_RLL_PIT | 姿态环增益 | 4.5 |
| ATC_RAT_RLL_P | 速率环Roll P | 0.15 |
| ATC_RAT_RLL_I | 速率环Roll I | 0.05 |
| ATC_RAT_RLL_D | 速率环Roll D | 0.003 |
| ATC_RAT_PIT_* | Pitch速率环参数 | (同Roll) |
| ATC_RAT_YAW_* | Yaw速率环参数 | (略不同) |

### 7.3 VTOL专用参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| Q_PILOT_SPD_UP | 上升速度限制 | 250 cm/s |
| Q_PILOT_SPD_DN | 下降速度限制 | 150 cm/s |
| Q_PILOT_ACCEL_Z | 垂直加速度 | 250 cm/s² |
| Q_THR_EXPO | 油门曲线指数 | 0.2 |

---

## 八、代码索引

### 8.1 核心文件

| 文件 | 说明 |
|------|------|
| [ArduPlane/quadplane.cpp](../ArduPlane/quadplane.cpp) | VTOL核心逻辑 |
| [ArduPlane/quadplane.h](../ArduPlane/quadplane.h) | VTOL类定义 |
| [ArduPlane/mode_qstabilize.cpp](../ArduPlane/mode_qstabilize.cpp) | QStabilize模式 |
| [ArduPlane/mode_qhover.cpp](../ArduPlane/mode_qhover.cpp) | QHover模式 |
| [ArduPlane/mode_qloiter.cpp](../ArduPlane/mode_qloiter.cpp) | QLoiter模式 |
| [libraries/AC_AttitudeControl/AC_PosControl.h](../libraries/AC_AttitudeControl/AC_PosControl.h) | 位置控制器 |
| [libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h](../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h) | 姿态控制器 |

### 8.2 关键函数

| 函数 | 位置 | 说明 |
|------|------|------|
| `hold_stabilize()` | quadplane.cpp:1061 | QStabilize核心控制 |
| `hold_hover()` | quadplane.cpp:1146 | QHover核心控制 |
| `vtol_position_controller()` | quadplane.cpp:2386 | 位置控制器总入口 |
| `assist_climb_rate_cms()` | quadplane.cpp:1479 | 自动爬升率计算 |
| `multicopter_attitude_rate_update()` | quadplane.cpp:960 | 姿态控制器调用 |
| `run_z_controller()` | AC_PosControl | 垂直位置环 |

---

## 九、总结

ArduPilot 的 VTOL 控制系统采用了经典的**层级控制架构**：

1. **任务层**：决定"去哪里"（航点、高度）
2. **位置层**：决定"多快到达"（速度、加速度）
3. **姿态层**：决定"怎么到达"（倾角、航向）
4. **执行层**：决定"电机怎么转"（推力差）

这种设计的核心优势：
- **解耦**：每层只关注一个物理量，便于调试
- **鲁棒**：下层失效时上层可以接管
- **灵活**：不同飞行模式复用同一套控制器

理解这套控制架构是深入学习 VTOL 飞控的基础，也是进行 VTOL 相关开发的关键。

---

*文档版本: 1.0*
*创建日期: 2024*
*基于 ArduPilot 源代码*
