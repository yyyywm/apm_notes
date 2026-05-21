# ArduPlane 代码架构 — 总索引

> 思维导图形式的总目录。点击各节点跳转到详细笔记。

---

## 一、文件地图

- **核心文件**
    - `Plane.h` — 主类定义，所有成员
    - `Plane.cpp` — 调度器 scheduler_tasks[]
    - `mode.h` / `mode.cpp` — Mode 基类 + 26 个派生类
    - `defines.h` — 常量/枚举
    - `Parameters.h` / `Parameters.cpp` — 参数表

- **子系统** → 详见各专题笔记
    - [姿态稳定与控制](04_姿态稳定与控制.md) — `Attitude.cpp`
    - [导航与航点](05_导航与航点.md) — `navigation.cpp` + `commands.cpp`
    - [舵机输出与混控](06_舵机输出与混控.md) — `servos.cpp`
    - RC 输入 — `radio.cpp` + `RC_Channel_Plane.cpp`
    - [失控保护体系](07_失控保护体系.md) — `events.cpp`
    - 启动与模式切换 — `system.cpp`
    - 数据日志 — `Log.cpp`
    - 空中调参 — `tuning.cpp`
    - 地面站通信 — `GCS_Plane.cpp` / `GCS_MAVLink_Plane.cpp`

- **飞行模式** → 详见 [飞行模式系统](02_飞行模式系统.md)
    - 固定翼: Manual / Stabilize / Training / Acro / FBWA / FBWB / Cruise / Auto / RTL / Guided / Loiter / Circle / Takeoff / AutoTune / Thermal / AutoLand / AvoidADSB
    - VTOL: QStabilize / QHover / QLoiter / QLand / QRTL / QAcro / QAutotune
    - 混合: LoiterAltQLand

- **VTOL** → 详见 [QuadPlane VTOL 子系统](03_QuadPlane_VTOL子系统.md)
    - `quadplane.h/.cpp` — QuadPlane 主类
    - `transition.h` — 过渡状态机 (SLT / Tailsitter / Tiltrotor)
    - `tailsitter.h` / `tiltrotor.h` — 特殊构型
    - `VTOL_Assist.h` — 固定翼模式 VTOL 辅助
    - `qautotune.h` — VTOL 自动调参

- **专项功能**
    - `soaring.cpp` — 热气流滑翔
    - `fence.cpp` — 电子围栏
    - `parachute.cpp` — 降落伞
    - `is_flying.cpp` — 飞行检测
    - `ekf_check.cpp` — EKF 健康检查
    - `reverse_thrust.cpp` — 反推

---

## 二、类关系概览

```
AP_Vehicle
  └── Plane (全局单例 plane)
        ├── QuadPlane (条件编译)
        │   ├── Transition* → SLT / Tailsitter / Tiltrotor
        │   ├── Tailsitter / Tiltrotor / VTOL_Assist
        │   └── AP_MotorsMulticopter* / AC_AttitudeControl_Multi* / AC_PosControl* ...
        ├── Mode* control_mode → 26 种飞行模式
        ├── AP_TECS / AP_L1_Control (导航控制器)
        ├── AP_RollController / AP_PitchController / AP_YawController (PID)
        ├── AP_Mission / AP_Landing
        └── AP_Camera / AP_Mount / AP_Parachute / AP_ADSB / AP_Rally / AP_Terrain ...
```

---

## 三、主循环调度器

→ 详见 [01_调度器与主循环](01_调度器与主循环.md)

```
FAST_TASK (每圈):
  1. ahrs_update()          — AHRS 姿态
  2. update_control_mode()  — 模式切换 + control_mode->update()
  3. stabilize()            — control_mode->run() → PID
  4. set_servos()           — PWM 输出

50Hz: read_radio / read_rangefinder / check_short_failsafe
10Hz: navigate / update_alt(TECS) / Log_Write_Fast
5Hz:  is_flying / crash_detect
3Hz:  check_long_failsafe
1Hz:  one_second_loop
```

---

## 四、飞行模式一览

→ 详见 [02_飞行模式系统](02_飞行模式系统.md)

| # | 模式 | 类型 | 特点 |
|---|------|------|------|
| 0 | MANUAL | 固定翼 | 纯手动 |
| 5 | FBWA | 固定翼 | 角度稳定 |
| 6 | FBWB | 固定翼 | 定高定速 |
| 10 | AUTO | 固定翼 | 自主任务 |
| 11 | RTL | 固定翼 | 返航 |
| 15 | GUIDED | 固定翼 | 外部控制 |
| 17 | QSTABILIZE | VTOL | VTOL 稳定 |
| 18 | QHOVER | VTOL | VTOL 悬停 |
| 19 | QLOITER | VTOL | VTOL 位置保持 |
| 20 | QLAND | VTOL | VTOL 降落 |
| 21 | QRTL | VTOL | VTOL 返航 |

---

## 五、QuadPlane VTOL

→ 详见 [03_QuadPlane_VTOL子系统](03_QuadPlane_VTOL子系统.md)

- 推力: SLT / TAILSITTER / TILTROTOR
- 位置状态机: NONE → APPROACH → AIRBRAKE → POSITION1 → POSITION2 → LAND_DESCEND → LAND_FINAL → COMPLETE
- 过渡: SLT (AIRSPEED_WAIT→TIMER→DONE) / Tailsitter (ANGLE_WAIT_FW→ANGLE_WAIT_VTOL→DONE)
- VTOL_Assist: Speed / Angle / Altitude / Force 四种辅助触发

---

## 六、定位问题快速导航

| 问题 | 入口 |
|------|------|
| 姿态抖动 | [姿态稳定](04_姿态稳定与控制.md) |
| 航线不对 | [导航](05_导航与航点.md) |
| 舵机/油门异常 | [舵机输出](06_舵机输出与混控.md) |
| 遥控/失控 | [失控保护](07_失控保护体系.md) |
| 模式切不过去 | `system.cpp` set_mode() |
| VTOL 过渡问题 | [QuadPlane](03_QuadPlane_VTOL子系统.md) |
| 参数不生效 | [参数系统](08_参数系统.md) |

---

## 七、调用追踪速查

**固定翼:**
```
scheduler → update_control_mode() → control_mode->update()  (导航层)
         → stabilize()           → control_mode->run()      (稳定层)
         → set_servos()          → PWM 输出
  navigate() (10Hz) → TECS/L1 → calc_nav_roll/pitch/throttle
```

**VTOL:**
```
control_mode->run() → quadplane.control_auto()
                    → quadplane.control_hover()
                    → quadplane.vtol_position_controller()
```
