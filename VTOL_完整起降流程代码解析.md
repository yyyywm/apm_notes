# 垂起（VTOL）完整起降流程代码解析

> 基于 ArduPlane QuadPlane 实现分析
> 核心文件：`ArduPlane/quadplane.cpp`、`ArduPlane/mode_q*.cpp`

---

## 一、核心架构概览

VTOL 的代码主要在 `ArduPlane/quadplane.cpp` 中实现，通过 `QuadPlane` 类管理。主循环每帧调用 `QuadPlane::update()`，这是整个 VTOL 逻辑的核心入口。

```
Plane::loop()
  └── QuadPlane::update()          [quadplane.cpp:1771]
        ├── 固定翼模式 → transition->update()   (状态机处理转换)
        └── VTOL模式   → motors_output()        (直接输出电机控制)
```

---

## 二、VTOL 模式定义

VTOL 飞行模式在 `ArduPlane/mode_q*.cpp` 中实现：

| 模式 | 文件 | 功能 |
|------|------|------|
| **QStabilize** | `mode_qstabilize.cpp` | 自稳模式，摇杆控制姿态 |
| **QHover** | `mode_qhover.cpp` | 定高模式，自动保持高度 |
| **QLoiter** | `mode_qloiter.cpp` | 定点悬停，自动保持位置 |
| **QLand** | `mode_qland.cpp` | 垂直降落 |
| **QRTL** | `mode_qrtl.cpp` | 返航并垂直降落 |

---

## 三、起飞流程（Takeoff）

### 1. 起飞入口 - `takeoff_controller()` [quadplane.cpp:3193]

```cpp
void QuadPlane::takeoff_controller(void)
{
    // 1. 安全检查：等待电机倾斜到位（倾转旋翼）
    if (tiltrotor.enabled() && !tiltrotor.fully_up()) {
        takeoff_start_time_ms = now;
        return;  // 等待电机转到垂直位置
    }

    // 2. 等待电调转速检查通过
    if (!motor_check_passed) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        return;  // 怠速等待
    }

    // 3. 等待方向舵回中（如果是方向舵解锁）
    if (!rc().seen_neutral_rudder()) {
        set_desired_spool_state(GROUND_IDLE);
        return;
    }

    // 4. 设置目标位置
    setup_target_position();

    // 5. 水平位置控制（可选：起飞初期不导航）
    if (no_navigation) {
        pos_control->NE_relax_velocity_controller();
    } else {
        pos_control->input_vel_accel_NE_m(vel_ne_ms, zero);
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();
    }

    // 6. 姿态控制
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(
        plane.nav_roll_cd, plane.nav_pitch_cd, yaw_rate);

    // 7. 垂直爬升控制
    set_climb_rate_ms(wp_nav->get_default_speed_up_ms());
    run_z_controller();  // 运行Z轴位置控制器
}
```

### 2. 起飞流程时序

```
[地面]
   ↓ 解锁，电机怠速 (GROUND_IDLE)
   ↓ 检查倾斜角度、转速、方向舵
   ↓ 电机解锁到全油门 (THROTTLE_UNLIMITED)
[垂直爬升]
   ↓ 位置控制器锁定当前位置
   ↓ Z轴控制器以设定速度上升
   ↓ 达到目标高度
[起飞完成] → 进入 QLoiter/QHover 悬停，或开始向前转换
```

---

## 四、降落流程（Landing）

降落是 VTOL 最复杂的流程，使用 **状态机** 管理，定义在 `quadplane.h:518`：

```cpp
enum position_control_state {
    QPOS_NONE = 0,
    QPOS_APPROACH,      // 固定翼方式接近降落点
    QPOS_AIRBRAKE,      // 旋翼当空气刹车减速
    QPOS_POSITION1,     // VTOL位置控制减速阶段
    QPOS_POSITION2,     // 水平定位到降落点正上方
    QPOS_LAND_DESCEND,  // 垂直下降
    QPOS_LAND_ABORT,    // 中止爬升
    QPOS_LAND_FINAL,    // 最终慢速下降到触地
    QPOS_LAND_COMPLETE  // 降落完成，锁定电机
};
```

### 完整降落状态机流程图

```
                    ┌─────────────────────────────────────────┐
                    │           QPOS_APPROACH                 │
                    │    (固定翼方式接近，使用TECS控制)         │
                    │    使用固定翼导航+油门，保持高度           │
                    └─────────────────┬───────────────────────┘
                                      │ 距离 < 停止距离
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │           QPOS_AIRBRAKE                 │
                    │    (旋翼电机启动作为空气刹车)             │
                    │    多旋翼电机启动产生阻力减速              │
                    └─────────────────┬───────────────────────┘
                                      │ 空速 < 阈值 或 姿态偏差大
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │           QPOS_POSITION1                │
                    │    (VTOL减速阶段，水平位置控制)           │
                    │    使用位置控制器减速到目标点              │
                    │    计算停止距离，逐步降低速度              │
                    └─────────────────┬───────────────────────┘
                                      │ 距离 < 10m 且 速度 < 3m/s
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │           QPOS_POSITION2                │
                    │    (精确定位到降落点正上方)               │
                    │    水平位置精确控制                       │
                    └─────────────────┬───────────────────────┘
                                      │ 水平位置到位
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │         QPOS_LAND_DESCEND               │
                    │    (开始垂直下降)                        │
                    │    以较快速度下降                         │
                    └─────────────────┬───────────────────────┘
                                      │ 高度 < Q_LAND_FINAL_ALT (默认6m)
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │          QPOS_LAND_FINAL                │
                    │    (最终慢速下降)                        │
                    │    使用 Q_LAND_FINAL_SPD (默认0.5m/s)    │
                    │    地面效应补偿                           │
                    └─────────────────┬───────────────────────┘
                                      │ 着陆检测通过 (4秒稳定)
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │         QPOS_LAND_COMPLETE              │
                    │    (降落完成，锁定电机)                   │
                    └─────────────────────────────────────────┘
```

### 关键代码：`vtol_position_controller()` [quadplane.cpp:2402]

这是降落状态机的核心实现：

```cpp
void QuadPlane::vtol_position_controller(void)
{
    switch (poscontrol.get_state()) {

    case QPOS_APPROACH:
        // 固定翼方式接近，使用TECS控制油门和俯仰
        plane.nav_controller->update_waypoint(...);
        SRV_Channels::set_output_scaled(k_throttle, TECS.get_throttle_demand());
        plane.nav_pitch_cd = TECS.get_pitch_demand();

        // 计算停止距离，决定何时进入AIRBRAKE
        if (distance_m < stop_distance) {
            poscontrol.set_state(QPOS_AIRBRAKE);
        }
        break;

    case QPOS_POSITION1:
        // VTOL减速阶段
        // 计算目标速度曲线，逐步减速
        approach_speed_ms = safe_sqrt(...);
        pos_control->input_vel_accel_NE_m(target_speed, target_accel);
        run_xy_controller();

        // 姿态控制
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(
            plane.nav_roll_cd, plane.nav_pitch_cd, target_yaw);

        // 进入POSITION2条件：距离<10m 且 速度<3m/s
        if (wp_distance < 10 && groundspeed < 3) {
            poscontrol.set_state(QPOS_POSITION2);
        }
        break;

    case QPOS_POSITION2:
    case QPOS_LAND_DESCEND:
        // 精确定位 + 垂直下降
        setup_target_position();
        pos_control->input_pos_vel_accel_NE_m(target_pos, target_vel, zero);
        run_xy_controller();
        break;

    case QPOS_LAND_FINAL:
        // 最终慢速降落
        // 接近地面时放松控制器
        if (should_relax()) {
            pos_control->NE_relax_velocity_controller();
        }
        break;
    }
}
```

### 着陆检测：`land_detector()` [quadplane.cpp:3585]

```cpp
bool QuadPlane::land_detector(uint32_t timeout_ms)
{
    // 条件1：应该放松控制（接近地面）且没有飞行员修正
    bool might_be_landed = should_relax() && !poscontrol.pilot_correction_active;

    // 条件2：垂直位置在4秒内变化不超过20cm
    if (fabsf(height_m - landing_detect.vpos_start_m) > 0.2) {
        return false;  // 高度还在变化
    }

    // 条件3：电机已在最低油门维持4秒+1秒
    if ((now - land_start_ms) < 4000 ||
        (now - lower_limit_start_ms) < 5000) {
        return false;  // 时间不够
    }

    return true;  // 确认着陆！
}
```

### 着陆完成处理：`check_land_complete()` [quadplane.cpp:3620]

```cpp
bool QuadPlane::check_land_complete(void)
{
    if (land_detector(4000)) {
        poscontrol.set_state(QPOS_LAND_COMPLETE);

        // 自动上锁（除非设置了继续任务）
        if (!plane.mission.continue_after_land()) {
            plane.arming.disarm(AP_Arming::Method::LANDED);
        }
        return true;
    }
    return false;
}
```

---

## 五、QLand 模式实现 [mode_qland.cpp]

```cpp
bool ModeQLand::_enter()
{
    // 复用QLoiter的初始化
    plane.mode_qloiter._enter();

    // 设置降落状态为开始下降
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);

    // 重置着陆检测
    quadplane.landing_detect.lower_limit_start_ms = 0;
    quadplane.landing_detect.land_start_ms = 0;

    return true;
}

void ModeQLand::run()
{
    // QLand 直接复用 QLoiter 的控制逻辑
    plane.mode_qloiter.run();
}
```

---

## 六、QRTL 返航降落 [mode_qrtl.cpp]

QRTL 有两个子模式：

```cpp
enum class SubMode {
    climb,   // 先爬升到安全高度
    RTL,     // 返航并降落
};
```

```cpp
bool ModeQRTL::_enter()
{
    // 1. 计算返航高度
    int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt_m * 100;

    // 2. 如果VTOL电机已在运行且距离近，直接VTOL返航
    if (motors_active && dist < radius) {
        poscontrol.set_state(QPOS_POSITION1);  // 直接VTOL模式
    }

    // 3. 否则先爬升，再固定翼返航，最后VTOL降落
    if (dist_to_climb > 0) {
        submode = SubMode::climb;  // 先爬升
    } else {
        submode = SubMode::RTL;    // 直接返航
    }
}

void ModeQRTL::run()
{
    switch (submode) {
    case SubMode::climb:
        // 垂直爬升到安全高度
        quadplane.set_climb_rate_ms(wp_nav->get_default_speed_up_ms());
        quadplane.run_z_controller();

        // 爬升完成后切换到RTL子模式
        if (stopping_point_reaches_target) {
            submode = SubMode::RTL;
        }
        break;

    case SubMode::RTL:
        // 调用vtol_position_controller执行完整降落状态机
        quadplane.vtol_position_controller();

        // 进入降落阶段后验证着陆
        if (poscontrol.get_state() >= QPOS_POSITION2) {
            quadplane.verify_vtol_land();
        }
        break;
    }
}
```

---

## 七、主循环调用关系

```
Plane::loop() [ArduPlane.cpp]
    ├── plane.read_radio()          ← 读取遥控器输入
    ├── plane.control_mode->update() ← 模式更新（设置nav_roll/nav_pitch目标）
    │       └── ModeQStabilize::update()
    │       └── ModeQLoiter::update()
    │       └── ...
    ├── plane.stabilize()           ← 固定翼姿态稳定
    ├── plane.navigate()            ← 导航计算
    └── quadplane.update()          ← VTOL核心更新 [quadplane.cpp:1771]
            ├── !in_vtol_mode() ?
            │       └── transition->update()   ← 固定翼模式：检查转换
            │           ├── 检查空速
            │           ├── 检查姿态
            │           └── 决定是否需要QAssist
            └── in_vtol_mode() ?
                    ├── motors_output()         ← 输出多旋翼电机
                    │       ├── attitude_control->rate_controller_run()
                    │       └── motors->output()
                    └── transition->VTOL_update()
```

---

## 八、关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Q_ENABLE` | 0 | 启用VTOL功能 |
| `Q_TRANSITION_MS` | 5000 | 转换时间（毫秒） |
| `Q_LAND_FINAL_ALT` | 6m | 最终降落阶段高度 |
| `Q_LAND_FINAL_SPD` | 0.5m/s | 最终降落速度 |
| `Q_PILOT_SPD_UP` | 2.5m/s | 最大上升速度 |
| `Q_PILOT_SPD_DN` | 0 | 最大下降速度 |
| `Q_RTL_ALT` | 15m | QRTL返航高度 |
| `Q_ASSIST_SPEED` | 0 | QAssist触发空速 |

---

## 九、总结

VTOL 的起降流程核心设计思想：

1. **起飞**：安全检查 → 怠速 → 位置锁定 → 垂直爬升，全程使用多旋翼控制器
2. **降落**：使用**8状态状态机**，从固定翼接近逐步过渡到垂直降落
3. **转换**：通过 `Transition` 类管理固定翼↔多旋翼的平滑切换
4. **安全**：多层检查（转速、姿态、高度稳定时间）确保着陆检测可靠
