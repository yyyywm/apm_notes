# 4+1垂起机型 AUTO模式从起飞到降落完整逻辑解析

## 一、整体架构与调用链

```
ArduPlane.cpp 主循环 (400Hz)
  └── Plane::set_servos()           [servos.cpp:861]
        └── quadplane.update()      [quadplane.cpp:1771]  ← 转换状态更新
              └── transition->update() / VTOL_update()

  └── ModeAuto::update()            [mode_auto.cpp:63]
        └── quadplane.control_auto() [quadplane.cpp:3359] ← VTOL飞行控制
              ├── takeoff_controller()   [quadplane.cpp:3190]  ← 起飞
              ├── waypoint_controller()  [quadplane.cpp:3314]  ← 巡航/航点
              └── vtol_position_controller() [quadplane.cpp:2399] ← 降落
```

---

## 二、起飞阶段 (VTOL Takeoff)

### 2.1 进入AUTO模式时初始化

[mode_auto.cpp:4-22](ArduPlane/mode_auto.cpp#L4-L22)
```cpp
bool ModeAuto::_enter() {
    // Q_ENABLE=2 时，AUTO模式默认进入VTOL模式
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    }
    plane.mission.start_or_resume();
}
```

### 2.2 任务航点触发 VTOL Takeoff

当任务执行到 `MAV_CMD_NAV_VTOL_TAKEOFF` 航点时：

[commands_logic.cpp](ArduPlane/commands_logic.cpp) → [quadplane.cpp:3421](ArduPlane/quadplane.cpp#L3421-L3481)
```cpp
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd) {
    // 设置目标高度（当前XY位置，只改变高度）
    plane.set_next_WP(loc);
    plane.next_WP_loc.set_alt_cm(plane.current_loc.alt + cmd.content.location.alt, ABSOLUTE);

    // 初始化垂直位置控制器
    pos_control->D_init_controller();

    // 计算预计起飞时间，用于起飞失败检测
    takeoff_start_time_ms = millis();
    takeoff_time_limit_ms = MAX(travel_time_s * takeoff_failure_scalar * 1000, 5000);
}
```

### 2.3 起飞控制循环

[quadplane.cpp:3190](ArduPlane/quadplane.cpp#L3190-L3309)
```cpp
void QuadPlane::takeoff_controller(void) {
    // 1. 等待解锁和安全检查
    if (!plane.arming.is_armed_and_safety_off()) return;

    // 2. 等待电机达到目标转速（ESC telemetry检查）
    // 3. 等待方向舵解锁后摇杆回中

    // 4. 设置目标位置
    setup_target_position();

    // 5. 水平位置控制（XY）
    pos_control->input_vel_accel_NE_m(vel_ne_ms, zero);
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();

    // 6. 姿态控制（调用多旋翼控制器）
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(
        plane.nav_roll_cd, plane.nav_pitch_cd, yaw_rate);

    // 7. 垂直速度控制（Z）
    float vel_u_ms = wp_nav->get_default_speed_up_ms();  // 默认上升速度
    set_climb_rate_ms(vel_u_ms);
    run_z_controller();
}
```

**起飞阶段特点**：
- 4个升力电机以多旋翼方式工作，产生垂直升力
- 推进电机（第5个电机）不工作
- 使用位置控制器保持XY位置，同时以设定速度爬升
- 可选 `Q_TKOFF_NAVALT_MIN` 参数控制低高度时不做导航

---

## 三、巡航/航点飞行阶段 (VTOL Waypoint)

当任务航点不是takeoff/land时，进入航点控制器：

[quadplane.cpp:3314](ArduPlane/quadplane.cpp#L3314-L3353)
```cpp
void QuadPlane::waypoint_controller(void) {
    // 1. 设置目标航点
    setup_target_position();
    wp_nav->set_wp_destination_NED_m(poscontrol.target_ned_m);

    // 2. 运行航点导航
    wp_nav->update_wpnav();

    // 3. 从航点控制器获取roll/pitch
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();

    // 4. 分配前向推力（如果使用推进电机辅助）
    assign_tilt_to_fwd_thr();

    // 5. 限制转换过程中的roll/pitch
    transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);

    // 6. 姿态控制
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(
        plane.nav_roll_cd, plane.nav_pitch_cd, wp_nav->get_yaw(), true);

    // 7. 高度控制
    set_climb_rate_ms(assist_climb_rate_cms() * 0.01);
    run_z_controller();
}
```

**巡航阶段特点**：
- 4个升力电机提供升力和姿态控制
- 推进电机（第5个）可提供前向推力辅助，减少多旋翼前倾需求
- 使用完整的 copter 航点导航（AC_WPNav）
- 支持 `Q_FWD_THR_GAIN` 参数配置前向推力增益

---

## 四、转换逻辑 (Transition)

### 4.1 前向转换 (VTOL → 固定翼)

在 `quadplane.update()` 中处理：

[quadplane.cpp:1818](ArduPlane/quadplane.cpp#L1818-L1843)
```cpp
void QuadPlane::update(void) {
    if (!in_vtol_mode() && !in_vtol_airbrake()) {
        // 处于固定翼模式，处理转换和辅助
        transition->update();   // SLT_Transition::update()
    } else {
        // 处于VTOL模式
        motors_output();
        transition->VTOL_update();
    }
}
```

**转换触发条件**：
- 空速达到 `ARSPD_FBW_MIN` 或 `Q_TRANSITION_MS` 超时
- 4+1机型：推进电机逐渐加速，多旋翼电机逐渐减速
- 转换完成后，完全由固定翼控制器接管

### 4.2 后向转换 (固定翼 → VTOL)

在降落前的APPROACH阶段触发：

[quadplane.cpp:2511](ArduPlane/quadplane.cpp#L2511-L2529)
```cpp
if (poscontrol.get_state() == QPOS_APPROACH && distance_m < stop_distance) {
    if (motors->get_desired_spool_state() == THROTTLE_UNLIMITED) {
        poscontrol.set_state(QPOS_POSITION1);
    } else {
        poscontrol.set_state(QPOS_AIRBRAKE);  // 先开多旋翼当空气刹车
    }
}
```

---

## 五、降落阶段 (VTOL Land)

降落是最复杂的阶段，分为多个子状态：

### 状态机定义

[quadplane.h:518](ArduPlane/quadplane.h#L518-L528)
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

### 5.1 APPROACH → AIRBRAKE → POSITION1

[quadplane.cpp:2435](ArduPlane/quadplane.cpp#L2435-L2573)
```cpp
case QPOS_APPROACH:
case QPOS_AIRBRAKE: {
    // 使用固定翼导航（TECS控制油门和俯仰）
    plane.nav_controller->update_waypoint(current_loc, loc);
    SRV_Channels::set_output_scaled(k_throttle, TECS_controller.get_throttle_demand());
    plane.nav_pitch_cd = TECS_controller.get_pitch_demand();
    plane.calc_nav_roll();

    // 计算停止距离，决定何时进入AIRBRAKE
    const float stop_distance = stopping_distance_m() + 2*closing_speed_ms;

    if (distance_m < stop_distance) {
        // 进入AIRBRAKE或POSITION1
    }

    // 空速过低时强制进入POSITION1
    if (aspeed_ms < aspeed_threshold_ms) {
        poscontrol.set_state(QPOS_POSITION1);
    }
}
```

### 5.2 POSITION1 (减速阶段)

[quadplane.cpp:2614](ArduPlane/quadplane.cpp#L2614-L2780)
```cpp
case QPOS_POSITION1: {
    // 计算目标减速速度
    const float stopping_speed_ms = safe_sqrt(
        MAX(0, wp_distance_m - 10.0) * 2 * transition_decel_mss + sq(3.0));

    // 输入目标速度和加速度到位置控制器
    pos_control->input_vel_accel_NE_m(target_speed_ne_ms, target_accel_ne_mss);
    pos_control->NE_stop_pos_stabilisation();  // 只控速度，不控位置

    run_xy_controller();

    // 获取roll/pitch并调用姿态控制器
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(...);

    // 接近目标点时进入POSITION2
    if (plane.auto_state.wp_distance < 10.0 && tiltrotor.tilt_angle_achieved()) {
        poscontrol.set_state(QPOS_POSITION2);
    }
}
```

### 5.3 POSITION2 (水平定位)

[quadplane.cpp:2783](ArduPlane/quadplane.cpp#L2783-L2817)
```cpp
case QPOS_POSITION2:
case QPOS_LAND_ABORT:
case QPOS_LAND_DESCEND: {
    // 使用完整位置控制（位置+速度）
    pos_control->input_pos_vel_accel_NE_m(poscontrol.target_ned_m.xy(), vel_ne_ms, zero);

    update_land_positioning();  // 允许飞行员摇杆修正降落位置

    run_xy_controller();

    // 姿态控制
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(...);
}
```

### 5.4 进入垂直下降 (LAND_DESCEND)

[quadplane.cpp:3673](ArduPlane/quadplane.cpp#L3673-L3755)
```cpp
bool QuadPlane::verify_vtol_land(void) {
    if (poscontrol.get_state() == QPOS_POSITION2) {
        // 检查是否到达降落点正上方且速度足够低
        if (reached_position && velocity < 3.0) {
            poscontrol.set_state(QPOS_LAND_DESCEND);
            pos_control->set_lean_angle_max_cd(0);  // 限制倾斜角为0
            // 放下起落架
        }
    }
}
```

### 5.5 LAND_DESCEND → LAND_FINAL

[quadplane.cpp:3650](ArduPlane/quadplane.cpp#L3650-L3668)
```cpp
bool QuadPlane::check_land_final(void) {
    // 高度低于 Q_LAND_FINAL_ALT（默认6m）且读数稳定
    if (height_above_ground_m < land_final_alt_m &&
        fabsf(height_above_ground_m - last_land_final_agl_m) < 5) {
        return true;  // 进入LAND_FINAL
    }
    // 或者着陆检测器触发
    return land_detector(6000);
}
```

### 5.6 LAND_FINAL (最终降落)

[quadplane.cpp:2819](ArduPlane/quadplane.cpp#L2819-L2859)
```cpp
case QPOS_LAND_FINAL:
    setup_target_position();
    update_land_positioning();

    if (should_relax()) {
        // 接近地面时放松位置控制
        pos_control->NE_relax_velocity_controller();
    } else {
        // 使用速度控制或位置控制
        pos_control->input_pos_vel_accel_NE_m(...);
    }

    run_xy_controller();

    // 姿态控制（允许飞行员摇杆修正航向）
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(...);
```

### 5.7 高度控制（Z轴）

[quadplane.cpp:2935](ArduPlane/quadplane.cpp#L2935-L2951)
```cpp
case QPOS_LAND_DESCEND:
case QPOS_LAND_ABORT:
case QPOS_LAND_FINAL: {
    float height_above_ground_m = plane.relative_ground_altitude(...);

    // LAND_FINAL阶段启用地面效应补偿
    if (poscontrol.get_state() == QPOS_LAND_FINAL) {
        ahrs.set_touchdown_expected(true);
    }

    // 根据高度计算下降速度
    const float descent_rate_ms = landing_descent_rate_ms(height_above_ground_m);
    pos_control->D_set_pos_target_from_climb_rate_ms(-descent_rate_ms, ...);
}
```

[quadplane.cpp:311](ArduPlane/quadplane.cpp#L311) 中的 `landing_descent_rate_ms`：
- 高于 `Q_LAND_FINAL_ALT`：使用 `Q_WP_SPEED_DN`
- 低于 `Q_LAND_FINAL_ALT`：使用 `Q_LAND_FINAL_SPD`（默认0.5m/s）

### 5.8 着陆检测与完成

[quadplane.cpp:3619](ArduPlane/quadplane.cpp#L3619-L3644)
```cpp
bool QuadPlane::check_land_complete(void) {
    if (poscontrol.get_state() != QPOS_LAND_FINAL) return false;

    // 着陆检测器判断（4秒内检测到着陆）
    if (land_detector(4000)) {
        poscontrol.set_state(QPOS_LAND_COMPLETE);

        // 自动解锁（除非MIS_OPTIONS设置继续任务）
        if (!plane.mission.continue_after_land()) {
            plane.arming.disarm(AP_Arming::Method::LANDED);
        }
        return true;
    }
    return false;
}
```

---

## 六、4+1机型特殊处理

### 6.1 前向推力分配

[quadplane.cpp:3022](ArduPlane/quadplane.cpp#L3022-L3076)
```cpp
void QuadPlane::assign_tilt_to_fwd_thr(void) {
    // 将俯仰需求映射到前向推力电机
    float fwd_tilt_rad = radians(constrain_float(-0.01f * plane.nav_pitch_cd, 0, 45));
    q_fwd_throttle = MIN(q_fwd_thr_gain * tanf(fwd_tilt_rad), 1.0);

    // 限制前俯仰角，防止机翼产生负升力
    q_fwd_pitch_lim_cd = ...
}
```

### 6.2 推进电机输出

[quadplane.cpp:1855](ArduPlane/quadplane.cpp#L1855-L1862)
```cpp
if (in_vtol_mode()) {
    float fwd_thr = 0;
    if (allow_forward_throttle_in_vtol_mode()) {
        fwd_thr = forward_throttle_pct();  // 计算前向推力百分比
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, fwd_thr);
}
```

---

## 七、总结流程图

```
+-----------------------------------------------------------------+
|                        AUTO 模式启动                             |
|              (Q_ENABLE=2 或 VTOL Takeoff航点)                     |
+-----------------------------------------------------------------+
                              |
                              v
+-----------------------------------------------------------------+
|  VTOL TAKEOFF                                                   |
|  * 4升力电机垂直起飞                                             |
|  * 推进电机关闭                                                  |
|  * 位置控制保持XY，速度控制Z                                      |
|  [takeoff_controller()]                                          |
+-----------------------------------------------------------------+
                              |
                              v
+-----------------------------------------------------------------+
|  VTOL WAYPOINT / 固定翼巡航                                       |
|  * 航点间飞行：waypoint_controller()                              |
|  * 可选前向推力辅助：assign_tilt_to_fwd_thr()                      |
|  * 或前向转换到固定翼模式                                         |
+-----------------------------------------------------------------+
                              |
                              v
+-----------------------------------------------------------------+
|  VTOL LAND                                                      |
|  +- QPOS_APPROACH: 固定翼方式接近                                  |
|  +- QPOS_AIRBRAKE: 多旋翼当空气刹车减速                             |
|  +- QPOS_POSITION1: VTOL减速到悬停速度                              |
|  +- QPOS_POSITION2: 水平定位到降落点正上方                           |
|  +- QPOS_LAND_DESCEND: 垂直下降 (Q_WP_SPEED_DN)                    |
|  +- QPOS_LAND_FINAL: 最终慢速下降 (Q_LAND_FINAL_SPD=0.5m/s)        |
|  +- QPOS_LAND_COMPLETE: 着陆检测 -> 自动解锁                         |
|  [vtol_position_controller()]                                    |
+-----------------------------------------------------------------+
```

---

## 八、关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Q_ENABLE` | 0 | 0=禁用, 1=启用, 2=VTOL AUTO |
| `Q_FRAME_CLASS` | 1 | 1=四轴 |
| `Q_FRAME_TYPE` | 1 | 1=X型 |
| `Q_TRANSITION_MS` | 5000 | 转换时间(ms) |
| `Q_LAND_FINAL_ALT` | 6m | 最终降落阶段高度 |
| `Q_LAND_FINAL_SPD` | 0.5m/s | 最终降落速度 |
| `Q_WP_SPEED_UP` | - | 上升速度 |
| `Q_WP_SPEED_DN` | - | 下降速度 |
| `Q_FWD_THR_GAIN` | 0 | 前向推力增益 |
| `Q_ASSIST_SPEED` | 0 | 辅助速度阈值 |

---

## 九、关键代码文件索引

| 文件 | 关键函数 | 说明 |
|------|----------|------|
| `ArduPlane/mode_auto.cpp` | `ModeAuto::update()` | AUTO模式主入口 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::control_auto()` | VTOL自动飞行控制分发 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::takeoff_controller()` | 起飞控制 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::waypoint_controller()` | 航点飞行控制 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::vtol_position_controller()` | 降落位置控制 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::update()` | 转换状态更新 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::verify_vtol_land()` | 降落阶段验证 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::check_land_final()` | 最终降落检查 |
| `ArduPlane/quadplane.cpp` | `QuadPlane::check_land_complete()` | 着陆完成检测 |
| `ArduPlane/servos.cpp` | `Plane::set_servos()` | 舵机/电机输出 |
| `ArduPlane/transition.cpp` | `SLT_Transition::update()` | 转换逻辑 |

---

> 生成时间: 2026-05-11
> 基于 ArduPlane 代码分析
