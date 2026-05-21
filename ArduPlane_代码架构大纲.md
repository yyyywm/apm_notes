# ArduPlane 代码架构大纲

> 思维导图形式梳理 ArduPlane (含 QuadPlane/VTOL) 代码结构，便于后续定位问题。

---

## 一、目录文件分类 (≈80个文件)

- **核心文件 (5)**
    - `Plane.h` — 主类，定义 Plane 及所有成员
    - `Plane.cpp` — 主循环调度器 scheduler_tasks[]
    - `mode.h` — Mode 基类 + 26个派生类声明
    - `mode.cpp` — Mode 基类 enter/exit/run 默认实现
    - `defines.h` — 所有关键常量/枚举 (failsafe, stick mixing, crash detection)
    - `Parameters.h` — 参数结构体声明 (g + g2 两组)
    - `Parameters.cpp` — 参数表 var_info[] 定义
    - `config.h` — 编译配置

- **子系统 (15)**
    - `system.cpp` — init_ardupilot() 启动初始化 + set_mode() 模式切换
    - `Attitude.cpp` — stabilize() 姿态PID (roll/pitch/yaw) + calc_speed_scaler()
    - `navigation.cpp` — navigate() 导航 + Loiter + 空速误差计算
    - `servos.cpp` — set_servos() 舵机输出 + 混控 (elevon/vtail/flaperon/spoiler)
    - `radio.cpp` — read_radio() RC输入 + 失控检测状态机
    - `RC_Channel_Plane.cpp` — do_aux_function() 辅助开关分发 (20+ 种 AUX 功能)
    - `commands.cpp` — 航点管理 (set_next_WP / set_guided_WP / update_home)
    - `commands_logic.cpp` — MAVLink 任务命令执行 (do_takeoff/nav_wp/land/change_speed...)
    - `events.cpp` — 失控处理 (failsafe_short/long/battery → 切模式动作)
    - `sensors.cpp` — 测距仪读取
    - `Log.cpp` — 数据日志 (CTUN/NTUN/QTUN/AETR/STAT...)
    - `tuning.cpp` — 遥控器空中调参 (AP_Tuning_Plane)
    - `GCS_Plane.cpp` — 地面站传感器状态上报
    - `GCS_MAVLink_Plane.cpp` — MAVLink 消息收发
    - `ArduPlane.cpp` — 残余代码，大部分已迁移到各子系统

- **飞行模式 (25个 mode_*.cpp)**
    - **固定翼模式**
        - `mode_manual.cpp` — 纯手动，直通舵机
        - `mode_stabilize.cpp` — 姿态稳定 + 飞手杆量
        - `mode_training.cpp` — 训练模式，限幅角度
        - `mode_acro.cpp` — 特技，角速度控制，四元数姿态
        - `mode_fbwa.cpp` — Fly-by-Wire A，角度稳定
        - `mode_fbwb.cpp` — Fly-by-Wire B，定高定速
        - `mode_cruise.cpp` — 巡航，锁定航向
        - `mode_auto.cpp` — 自主任务 (mission.update)
        - `mode_rtl.cpp` — 返航 (爬升→盘旋→进场→降落)
        - `mode_guided.cpp` — 地面站/外部控制
        - `mode_loiter.cpp` — 当前位置盘旋
        - `mode_circle.cpp` — 自动盘旋
        - `mode_takeoff.cpp` — 自主起飞，有自己的参数组
        - `mode_autotune.cpp` — 自动PID调参
        - `mode_thermal.cpp` — 热气流滑翔
        - `mode_autoland.cpp` — 自主降落，有自己的参数组
        - `mode_avoidADSB.cpp` — ADSB 碰撞规避 (is_guided_mode)
    - **VTOL 模式**
        - `mode_qstabilize.cpp` — VTOL 姿态稳定
        - `mode_qhover.cpp` — VTOL 悬停
        - `mode_qloiter.cpp` — VTOL 位置保持
        - `mode_qland.cpp` — VTOL 降落，着陆状态机
        - `mode_qrtl.cpp` — VTOL 返航 (RTL→LOITER_TO_ALT→进场→降落)
        - `mode_qacro.cpp` — VTOL 特技，手动油门
        - `mode_qautotune.cpp` — VTOL 自动调参
    - **混合模式**
        - `mode_LoiterAltQLand.cpp` — 继承 ModeLoiter，Loiter + QLand 备降

- **VTOL / 过渡 (6)**
    - `quadplane.h` — QuadPlane 类声明，拥有所有 VTOL 子系统
    - `quadplane.cpp` — QuadPlane 主逻辑 (control_auto / vtol_position_controller / update)
    - `transition.h` — Transition 抽象基类 + SLT_Transition 状态机 (AIRSPEED_WAIT→TIMER→DONE)
    - `tailsitter.h` — Tailsitter 类 + Tailsitter_Transition (ANGLE_WAIT_FW→ANGLE_WAIT_VTOL→DONE)
    - `tiltrotor.h` — Tiltrotor 类 + Tiltrotor_Transition (继承 SLT_Transition)
    - `VTOL_Assist.h` — 固定翼模式下 VTOL 辅助 (Speed/Angle/Altitude/Force 四种 Assist)
    - `qautotune.h` — QuadPlane 自动调参

- **专项功能 (剩余)**
    - `soaring.cpp` — 热气流滑翔逻辑
    - `pullup.h` — 自动拉升/避障
    - `systemid.h` — 系统辨识
    - `AP_Arming_Plane.h` — 上锁/解锁
    - `fence.cpp` — 电子围栏
    - `parachute.cpp` — 降落伞
    - `motor_test.cpp` — 电机测试
    - `is_flying.cpp` — 飞行状态检测
    - `ekf_check.cpp` — EKF 健康检查
    - `reverse_thrust.cpp` — 反推
    - `avoidance_adsb.cpp` — ADSB 规避

---

## 二、核心类继承与组合关系

- **AP_Vehicle** (libraries/AP_Vehicle) — ArduPilot 车辆基类
    - **Plane** (ArduPlane/Plane.h) — 全局单例 `plane`
        - 组合 QuadPlane `quadplane` — VTOL 管理 (条件编译 `HAL_QUADPLANE_ENABLED`)
            - 组合 Transition* `transition` — 过渡状态机 (多态基类指针)
                - `SLT_Transition` — 独立升力推力过渡
                - `Tailsitter_Transition` — 尾座式过渡
                - `Tiltrotor_Transition` — 倾转旋翼过渡
            - 组合 Tailsitter `tailsitter` — 尾座式控制
            - 组合 Tiltrotor `tiltrotor` — 倾转旋翼控制
            - 组合 VTOL_Assist `assist` — 固定翼模式辅助
            - 组合 QAutoTune `qautotune` — VTOL 自动调参
            - 组合 AP_MotorsMulticopter* `motors` — 多旋翼电机混控
            - 组合 AC_AttitudeControl_Multi* `attitude_control` — VTOL 姿态控制
            - 组合 AC_PosControl* `pos_control` — VTOL 位置控制
            - 组合 AC_WPNav* `wp_nav` — VTOL 航点导航
            - 组合 AC_Loiter* `loiter_nav` — VTOL 悬停导航
        - 组合 Mode* `control_mode` — 当前飞行模式指针
            - **Mode** (基类，mode.h) — 虚方法: enter/exit/update/run/navigate
                - 固定翼: Manual / Stabilize / Training / Acro / FBWA / FBWB / Cruise / Auto / RTL / Guided / Loiter / Circle / Takeoff / AutoTune / Thermal / AutoLand / AvoidADSB
                - VTOL: QStabilize / QHover / QLoiter / QLand / QRTL / QAcro / QAutotune
                - 混合: LoiterAltQLand (继承 ModeLoiter)
        - 组合 AP_TECS `TECS_controller` — 总能量控制 (高度/速度耦合)
        - 组合 AP_L1_Control `L1_controller` — L1 横向导航
        - 组合 AP_RollController / AP_PitchController / AP_YawController / AP_SteerController — PID 控制器
        - 组合 AP_Mission `mission` — 任务系统
        - 组合 AP_Landing `landing` — 降落管理
        - 组合 AP_Camera / AP_Mount / AP_Parachute / AP_ADSB / AP_Rally / AP_Terrain ... — 外围子系统

---

## 三、主循环调度器

- **scheduler_tasks[]** (Plane.cpp) — 按优先级排列的任务表
    - **FAST_TASK (每圈都跑)**
        - 1. `ahrs_update()` — AHRS 姿态更新
        - 2. `update_control_mode()` — 检查模式切换，调用 `control_mode->update()`
        - 3. `stabilize()` — 调用 `control_mode->run()` → PID 稳定
        - 4. `set_servos()` — 输出 PWM 到舵机/电机
    - **50Hz:** `read_radio()` / `read_rangefinder()` / `check_short_rc_failsafe()`
    - **10Hz:** `navigate()` / `update_alt()` → TECS / `Log_Write_Fast()`
    - **5Hz:** `is_flying` 检测 / `crash` 检测
    - **3Hz:** `check_long_failsafe()` / `gcs_send_heartbeat()`
    - **1Hz:** `one_second_loop()` (通道映射 / 更新 home / ADSB / 罗盘累积)
    - **其他:** GPS 更新 (50/10Hz) / 日志写入 / fence 检查 / ADSB 规避
- **每圈控制流**
    - `read_radio()` → `update_control_mode()` → `stabilize()` → `set_servos()`
    - 10Hz 补充: `navigate()` → `update_alt()` (在 stabilize 之前)

---

## 四、飞行模式系统

- **Mode 基类** (mode.h) — 所有飞行模式的抽象基类
    - **生命周期方法**
        - `enter()` / `_enter()` — 模式切换时调用 (初始化: 重置标志位 / 设置目标值)
        - `exit()` / `_exit()` — 模式退出时调用 (清理/保存状态)
    - **运行方法 (由调度器驱动)**
        - `update()` — FAST_TASK 每圈: 模式特定的**导航层**逻辑 (计算导航需求)
        - `run()` — FAST_TASK 每圈: 模式特定的**稳定层**逻辑 (PID 控制)
        - `navigate()` — 10Hz: 航点追踪 (仅 auto/rtl/guided 等有自动导航的模式)
    - **能力查询方法**
        - `is_vtol_mode()` / `is_guided_mode()` / `does_auto_navigation()` / `does_auto_throttle()`
        - `allows_throttle_nudging()` / `allows_terrain_disable()` / `is_landing()` / `is_taking_off()`
    - **run() 默认流程**
        - 1. STICK_MIXING 混控 (NONE / FBW / VTOL_YAW / FBW_NO_PITCH)
        - 2. `stabilize_roll()` → 滚转 PID → 副翼
        - 3. `stabilize_pitch()` → 俯仰 PID → 升降
        - 4. `stabilize_yaw()` → 偏航 PID (协调转弯 / 航向保持 / 地面转向)

- **固定翼模式** (Mode::Number 枚举)
    - `0` MANUAL — 纯手动，直通舵机，无稳定
    - `1` CIRCLE — 自动盘旋 (auto_navigation + auto_throttle)
    - `2` STABILIZE — 姿态稳定，飞手控制油门
    - `3` TRAINING — 训练模式，限幅角度
    - `4` ACRO — 特技，角速度控制，四元数姿态
    - `5` FBWA — Fly-by-Wire A，角度稳定
    - `6` FBWB — Fly-by-Wire B，定高定速
    - `7` CRUISE — 巡航，锁定航向，飞手油门 = trim
    - `8` AUTOTUNE — 自动 PID 调参
    - `10` AUTO — 自主任务 (mission.update)，最复杂的模式
    - `11` RTL — 返航 (爬升→返航高度→盘旋→进场→降落)
    - `12` LOITER — 当前位置盘旋
    - `13` TAKEOFF — 自主起飞，有自己的参数组
    - `14` AVOID_ADSB — ADSB 碰撞规避 (is_guided_mode)
    - `15` GUIDED — 地面站/外部控制 (handle_guided_request)
    - `16` INITIALISING — 启动占位 (_enter 返回 false)
    - `24` THERMAL — 热气流滑翔
    - `26` AUTOLAND — 自主降落，有自己的参数组

- **VTOL 模式** (is_vtol_mode = true)
    - `17` QSTABILIZE — VTOL 姿态稳定
    - `18` QHOVER — VTOL 悬停
    - `19` QLOITER — VTOL 位置保持 (有自己的 pos_control)
    - `20` QLAND — VTOL 降落 (着陆状态机)
    - `21` QRTL — VTOL 返航 (RTL→LOITER_TO_ALT→进场→降落)
    - `22` QAUTOTUNE — VTOL 自动调参
    - `23` QACRO — VTOL 特技 (手动油门)

- **混合模式**
    - `25` LOITER_ALT_QLAND — 继承 ModeLoiter，Loiter + QLand 备降

---

## 五、QuadPlane / VTOL 子系统

- **推力类型** (ThrustType 枚举)
    - SLT — 独立升力推力 (前拉电机 + 四轴电机分开，最常见)
    - TAILSITTER — 尾座式 (机身竖立)
    - TILTROTOR — 倾转旋翼 (电机可倾转)

- **位置控制状态机** (position_control_state)
    - `QPOS_NONE → QPOS_APPROACH → QPOS_AIRBRAKE → QPOS_POSITION1 → QPOS_POSITION2 → QPOS_LAND_DESCEND → QPOS_LAND_FINAL → QPOS_LAND_COMPLETE`
    - 特殊: `QPOS_LAND_ABORT` — 中止降落，回到悬停

- **降落进场阶段** (VTOLApproach::Stage)
    - `RTL → LOITER_TO_ALT → ENSURE_RADIUS → WAIT_FOR_BREAKOUT → APPROACH_LINE → VTOL_LANDING`

- **过渡状态机** (固定翼 ↔ VTOL)
    - SLT_Transition: `AIRSPEED_WAIT → TIMER → DONE`
    - Tailsitter_Transition: `ANGLE_WAIT_FW → ANGLE_WAIT_VTOL → DONE`
    - Tiltrotor_Transition: 继承 SLT_Transition，覆盖 yaw/view/vfwd 逻辑

- **VTOL_Assist** (固定翼模式下的四旋翼辅助)
    - Speed Assist — 空速 < Q_ASSIST_SPEED → 启动四旋翼
    - Angle Assist — 姿态误差 > Q_ASSIST_ANGLE → 启动四旋翼
    - Altitude Assist — 检测到掉高 → 启动四旋翼
    - Force Assist — 遥控器开关强制启用

- **关键参数** (Q_* 系列)
    - `Q_ENABLE` — 主开关 (0:关, 1:开, 2:VTOL AUTO)
    - `Q_FRAME_CLASS` — 机架类型 (Quad/Hexa/Octa/Tri/Y6)
    - `Q_FRAME_TYPE` — 机架布局 (X/+/H/V)
    - `Q_TRANSITION_MS` — 过渡时间 ms
    - `Q_PILOT_SPD_UP/DN` — 飞手升降速率
    - `Q_LAND_FINAL_SPD/ALT` — 降落的末段速度/高度
    - `Q_ASSIST_SPEED/ANGLE/ALT/DELAY` — 辅助触发条件
    - `Q_VFWD_GAIN` — 前向速度保持增益
    - `Q_OPTIONS` — 23+ 位掩码选项

---

## 六、核心子系统详解

- **Attitude.cpp** — 姿态稳定
    - `stabilize()` → 调用 `control_mode->run()` (FAST_TASK 入口)
    - `calc_speed_scaler()` — 基于空速的动态 PID 增益缩放
    - `stabilize_roll()` — 滚转 PID → 副翼通道
    - `stabilize_pitch()` — 俯仰 PID → 升降通道 (倒飞/VTOL从属/起飞拉杆/flare)
    - `stabilize_yaw()` — 偏航三模式: 地面转向(航向) / 地面转向(角速度) / 协调转弯
    - `calc_nav_roll()` — 导航需求 → 滚转限制值
    - `calc_nav_pitch()` — 导航需求 → 俯仰限制值
    - `calc_throttle()` — 油门需求计算
    - `apply_load_factor_roll_limits()` — 载荷因子滚转限制 (防失速)

- **navigation.cpp** — 导航
    - `navigate()` — 10Hz，计算航点距离/方位，调用 `control_mode->navigate()`
    - `update_loiter()` — Loiter 导航 (半径+方向 → L1 控制器)
    - `calc_airspeed_errors()` — 空速目标 (多源: FBWB杆/滑翔/降落/AUTO) + 地速补偿
    - `calc_gndspeed_undershoot()` — 地速不足补偿
    - `update_fbwb_speed_height()` — FBWB/CRUISE 高度控制 (升降控高度，油门控速度)

- **servos.cpp** — 舵机输出
    - `set_servos()` — FAST_TASK，主输出入口
    - `set_throttle()` — 油门限制 / 电压补偿 / 抑制逻辑
    - `servos_output()` — 混控输出 (elevon / vtail / flaperon / spoiler / twin-engine)
    - `throttle_slew_limit()` — 油门变化率限制
    - `suppress_throttle()` — 油门抑制判定 (地面/降落/降落伞)
    - `set_servos_flaps()` — 襟翼混控
    - `dspoiler_update()` — 差动扰流板
    - `servos_auto_trim()` — 自动微调 (监控 I 项 → 逐步写入 trim → 每10秒保存 FRAM)

- **radio.cpp** — RC 遥控输入
    - `read_radio()` — 50Hz，读取 RC + 失控检测
    - `set_control_channels()` — 1Hz，映射 roll/pitch/throttle/rudder 通道
    - `control_failsafe()` — 失控状态机 (NONE → SHORT)
    - `trim_radio()` — 自动配平 (副翼/升降/方向/elevon/vtail)
    - `rudder_input()` — 方向舵，含 rudder_only 和 DIRECT_RUDDER_ONLY 选项

- **events.cpp** — 失控处理
    - `failsafe_short_on_event()` — 短时失控 → FBWA/FBWB/CIRCLE/QLAND
    - `failsafe_short_off_event()` — 短时失控恢复
    - `failsafe_long_on_event()` — 长时失控 → RTL/GLIDE/PARACHUTE/AUTO/AUTOLAND
    - `failsafe_long_off_event()` — 长时失控恢复
    - `handle_battery_failsafe()` — 电池失控 → QLand/LoiterAltQLand/Land/RTL/Terminate/Parachute

- **system.cpp** — 启动与模式切换
    - `init_ardupilot()` — 主初始化 (INS/RC/Baro/GPS/Compass/Mission/Logger...)
    - `startup_INS()` — 启动 AHRS，设置 vehicle_class=FIXED_WING
    - `set_mode()` — 中心模式切换 (验证 → enter → exit → 日志)
    - `check_long_failsafe()` — 3Hz，监控 RC/GCS 心跳

- **commands.cpp** — 航点管理
    - `set_next_WP()` — 加载下一个航点 (含 crosstrack / alt_slope / turn_angle)
    - `set_guided_WP()` — 设置 Guided 模式目标
    - `update_home()` — 自动更新 home 位置

- **commands_logic.cpp** — MAVLink 任务命令
    - `do_takeoff()` / `do_nav_wp()` / `do_land()` / `do_change_speed()` / `do_change_alt()`
    - `do_vtol_transition()` / `do_nav_delay()` / `do_set_roi()` ...

- **Log.cpp** — 数据日志
    - CTUN — 控制调参 (nav_roll/pitch, roll/pitch, throttle_out, airspeed)
    - NTUN — 导航调参 (wp_distance, bearing, alt_error, crosstrack, airspeed_error)
    - QTUN — 四旋翼调参 (quat, rate, accel targets)
    - AETR — 执行器输出 (aileron/elevator/throttle/rudder 归一化)
    - STAT — 状态 (flight stage, failsafe, nav state)

- **RC_Channel_Plane.cpp** — AUX 开关
    - `do_aux_function()` — 分发 20+ 种辅助功能:
    - INVERTED / REVERSE_THROTTLE / AUTO / CIRCLE / LOITER / GUIDED / RTL / FBWA / QRTL
    - Q_ASSIST / CROW_SELECT / TER_DISABLE / LANDING_FLARE / FW_AUTOTUNE / SOARING / AIRMODE

- **defines.h** — 关键常量
    - Failsafe 状态: NONE=0 / SHORT=1 / LONG=2 / GCS=3
    - Short FS 动作: BESTGUESS(CIRCLE) / CIRCLE / FBWA / DISABLED / FBWB
    - Long FS 动作: CONTINUE / RTL / GLIDE / PARACHUTE / AUTO / AUTOLAND
    - Stick Mixing: NONE / FBW / VTOL_YAW / FBW_NO_PITCH
    - FlightOptions: 17 个位选项 (DIRECT_RUDDER_ONLY / CLIMB_BEFORE_TURN / ACRO_YAW_DAMPER ...)
    - Crash Detection: DISABLED / DISARM
    - AirMode: OFF / ON / ASSISTED_FLIGHT_ONLY
    - Crow Flap: FLYINGWING / FULLSPAN / PROGRESSIVE_CROW

---

## 七、定位问题快速导航

- **飞机飞不稳 / 姿态抖动** → `Attitude.cpp` (PID) + `mode.h` (对应模式的 run)
- **航点 / 航线不对** → `navigation.cpp` + `commands.cpp`
- **舵机不动 / 油门异常** → `servos.cpp` (输出/混控/抑制)
- **遥控器不响应 / 失控** → `radio.cpp` + `events.cpp`
- **模式切不过去** → `system.cpp` set_mode() + 对应 `mode_*.cpp` _enter()
- **垂直起降过渡问题** → `quadplane.cpp` control_auto() + `transition.h`
- **VTOL 降落位置不准** → `quadplane.cpp` vtol_position_controller()
- **Q_ASSIST 误触发** → `VTOL_Assist.cpp`
- **参数不生效** → `Parameters.h` (索引) + `Parameters.cpp`
- **日志缺数据** → `Log.cpp` Log_Write_* 系列
- **自动起飞 / 自动降落异常** → `mode_takeoff.cpp` / `mode_autoland.cpp`
- **电池失控保护误触发** → `events.cpp` handle_battery_failsafe()
- **Fence 误触发** → `fence.cpp` + `defines.h` FenceAutoEnable
- **Soaring 逻辑问题** → `soaring.cpp` + `mode_thermal.cpp`
- **TECS 高度/速度控制问题** → `Plane.cpp` update_alt() + TECS_controller
- **L1 横向导航问题** → `navigation.cpp` L1_controller

---

## 八、总结: 三层架构 + 调用追踪路径

- **三层架构**
    - 1. 调度器层 — `Plane.cpp` scheduler_tasks[] 控制所有任务的执行顺序和频率
    - 2. 模式系统 — `mode.h` Mode 基类 + 26 个派生类，各自封装导航+稳定逻辑
    - 3. VTOL 子系统 — `quadplane.h` QuadPlane 类，条件编译透明叠加 VTOL 能力

- **调用追踪通用路径 (固定翼)**
    - `scheduler_tasks[]` → `control_mode->update()` → `control_mode->run()`
    - ↑ 10Hz: `navigate()` → TECS / L1 → `calc_nav_roll/pitch/throttle`
    - ↑ FAST_TASK: `stabilize_roll/pitch/yaw()` → 舵机 PWM

- **调用追踪路径 (VTOL 模式)**
    - `control_mode->run()` → `quadplane.control_auto()` / `control_hover()` / `vtol_position_controller()`
