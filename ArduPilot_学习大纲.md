# ArduPilot 学习大纲（固定翼 & 垂起/VTOL）

> 视角：以固定翼（Plane）和垂直起降固定翼（QuadPlane/VTOL）为主线，串联所有模块。

---

## 学习方法论

| 原则 | 一句话 | 行动准则 |
|------|--------|----------|
| **第一性原理** | 回到物理，不是回到代码 | 读代码时自问："这段代码在对抗什么物理现实？" |
| **费曼学习法** | 能一句话讲清楚才算懂 | 每模块看完，不看代码用一句话写下它做什么 |
| **系统1/2** | 能看懂语法 ≠ 能理解逻辑 | 系统1扫结构，系统2追踪完整数据流。不要骗自己 |
| **成长型思维** | 不懂处是学习信号，不是能力证明 | 标记它，追问它，不跳过 |
| **拉伸区** | 80% 时间花在"有点难但能搞定"的内容 | C++语法是舒适区不产生复利，ChibiOS内核暂不碰 |
| **认知复利** | 每理解一个节点，下一个节点附着成本减半 | 优先建立核心节点连接 |

---

## 核心认知：一架垂起飞机有两套"大脑"

QuadPlane 的本质不是"多了几个电机的固定翼"，而是**一架飞机体内住着两个飞控**：

```
            ┌─ VTOL模式 ─────────────────────────┐
            │  Copter 的控制架构                    │
            │  AC_PosControl → AC_AttControl_Multi │
            │  → AP_MotorsMatrix → 4个升力电机      │
            │                                      │
  传感器 ──┤                                      ├── 舵机/电机
 (共享)    │                                      │
            │  固定翼模式 ──────────────────────── │
            │  Plane 的控制架构                     │
            │  TECS(速度+高度) + L1(横向导航)       │
            │  → 舵面 + 推进电机                    │
            └──────────────────────────────────────┘

            ↑────── Transition 是两者之间的桥 ──────↑
```

**这意味着**：学习 QuadPlane 等于同时学习 Copter 的 VTOL 侧 + Plane 的固定翼侧 + Transition 转换逻辑。这是 ArduPilot 里最复杂的机型。

---

## 贯穿主线：两种飞行模式的完整数据流

### 主线 A：固定翼巡航（FBWA 模式，50Hz 主循环）

```
GPS/IMU/Baro → EKF → TECS(俯仰/油门) → 舵面混控 → 伺服输出
                      L1(滚转导航)   ↗
```

### 主线 B：VTOL 悬停（QHOVER 模式，300Hz 主循环）

```
GPS/IMU/Baro → EKF → AC_PosControl → AC_AttControl_Multi → AP_MotorsMatrix → 电机PWM
```

> 注：固定翼硬件默认主循环 50Hz，QuadPlane 重写为 300Hz（`ArduPlane/Parameters.cpp:1368`）。SITL 仿真环境默认 400Hz。4 个 FAST_TASK（ahrs_update / update_control_mode / stabilize / set_servos）在每个循环都执行，其余任务按调度频率执行（如导航 10Hz、空速计算 10Hz、高度控制 50Hz）。本文按模式区分速率。

---

## 模块地图总览

```
┌──────────────────────────────────────────────────────────┐
│  传感器层（固定翼/VTOL 共享）                               │
│  IMU · Baro · GPS · Compass · Airspeed                   │
├──────────────────────────────────────────────────────────┤
│  状态估计层                                               │
│  AHRS · EKF2/3                                            │
├───────────────────────┬──────────────────────────────────┤
│  固定翼控制（Plane）    │  VTOL控制（QuadPlane）            │
│                       │                                  │
│  TECS (总能量)         │  AC_PosControl (位置→速度→倾角)   │
│  └ 速度=油门           │  AC_AttitudeControl_Multi (姿态)  │
│  └ 高度=俯仰           │                                  │
│  L1 (横向导航)          │  飞行模式(Q*)                     │
│  └ 滚转=航向修正        │  └ 悬停/定点/返航                 │
│                       │                                  │
│  飞行模式(固定翼)       │  QuadPlane主类                    │
│  └ FBWA/FBWB/CRUISE    │  └ 电机管理/转换/辅助             │
│  └ AUTO/RTL/LOITER     │                                  │
├───────────────────────┴──────────────────────────────────┤
│  过渡层：Transition                                       │
│  SLT_Transition · Tailsitter_Transition                   │
├──────────────────────────────────────────────────────────┤
│  执行层                                                   │
│  AP_Motors(电机+舵面) · SRV_Channel(硬件输出)              │
├──────────────────────────────────────────────────────────┤
│  横切面：AP_Param(参数) · AP_Logger(日志)                  │
└──────────────────────────────────────────────────────────┘
```

---

## Part 1：传感器层（固定翼与 VTOL 共享）

### 1.1 惯性传感器 `AP_InertialSensor`
- 读什么：加速度（m/s²）+ 角速度（rad/s），三轴
- 核心：`AP_InertialSensor.h/cpp`（主类）、`AP_InertialSensor_Backend.h/cpp`（驱动抽象）
- 固定翼特别关注：振动环境比多旋翼更复杂（发动机振动+气动颤振），陷波滤波（Harmonic Notch）尤为重要

### 1.2 空速计 — 固定翼独有
- 读什么：空速（m/s），分指示空速（IAS）和真实空速（TAS）
- 核心文件：`AP_Airspeed`（`libraries/AP_Airspeed/`）
- **固定翼为什么必须关注空速**：固定翼靠机翼产生升力，升力∝空速²。地速≠空速（有风时）。TECS 和 L1 都依赖空速。失速保护依赖空速。

### 1.3 GPS `AP_GPS`
- 读什么：经纬度、高度、地速、航向、精度
- 核心：`AP_GPS.h/cpp`、`AP_GPS_Blended.h/cpp`
- 固定翼场景：GPS 提供地速用于风估算（风速 = 空速矢量 - 地速矢量），这是固定翼特有的计算

### 1.4 气压计 `AP_Baro`
- 读什么：气压 → 高度；温度补偿
- 核心：`AP_Baro.h/cpp`、`AP_Baro_Backend.h/cpp`
- 固定翼场景：高度控制是 TECS 的核心输入之一

### 1.5 磁罗盘 `Compass`
- 读什么：三轴磁场 → 磁北方向
- 核心：`AP_Compass.h/cpp`（类名 `Compass`）、校准在 `AP_Compass_Calibration.cpp`

---

## Part 2：状态估计（共享）

### AHRS `AP_AHRS`
- 输出：姿态四元数 + roll/pitch/yaw + 速度 + 位置
- 核心文件：`AP_AHRS.h/cpp`、`AP_AHRS_Backend.h/cpp`
- 后端：`AP_AHRS_NavEKF2/3`（EKF 封装）、`AP_AHRS_DCM`（互补滤波，老方案，EKF 失效时备用）

### EKF `AP_NavEKF2` / `AP_NavEKF3`
- 两个 EKF 每个主循环都在跑，取其优
- EKF2：28 状态（含陀螺刻度因子和单轴加速度偏置）
- EKF3：24 状态（数值精度更高，独有 drag fusion、body-odometry）
- 共享特性：ext_nav（外部导航）、GSF yaw 估计器
- 固定翼特有融合：空速融合（`AirDataFusion.cpp`）、风速估算（2 状态）
- 延迟补偿：GPS 延迟 ~200ms，EKF 用环形缓冲区倒推修正，上限 250ms

---

## Part 3A：固定翼控制链

> 第一性原理：固定翼控制的本质是**能量管理**。飞机的总能量 = 动能（速度）+ 势能（高度）。油门控制总能量的大小，俯仰控制能量在动能和势能之间的分配。这与多旋翼的"油门=升力=高度"完全不同。

### TECS — 总能量控制系统 `AP_TECS`
- **核心文件**：`libraries/AP_TECS/AP_TECS.h/cpp`
- 做什么：同时控制**速度**和**高度**，通过协调油门和俯仰来实现
  - 总能量误差 → **油门**（动能+势能都不够 → 加油门；都多余 → 收油门）
  - 能量分配误差 → **俯仰**（速度够但高度不够 → 抬头，动能转势能；反之低头）
- 输入：目标高度、目标空速、当前高度、当前空速
- 输出：油门百分比（-100~100）、目标俯仰角
- 关键参数：时间常数、阻尼比、最大/最小油门和俯仰限制
- 固定翼与多旋翼的根本区别就在这里——固定翼不能 hover，速度太低会失速

### L1 导航控制 `AP_L1_Control`
- **核心文件**：`libraries/AP_L1_Control/AP_L1_Control.h/cpp`
- 做什么：横向导航——计算飞机应该滚转多少度来跟踪目标航线
- 原理：在目标航线上前方 L1 距离处选一个参考点，计算指向该点需要的横向加速度，再换算为滚转角
- 输入：飞机当前位置、目标航线（prev → next waypoint）
- 输出：目标滚转角（`nav_roll_cd`）
- 也支持：悬停绕圈（loiter）、航向保持（heading hold）

### Plane 姿态控制
- **核心文件**：`ArduPlane/Attitude.cpp`，控制器来自 `libraries/APM_Control/`
- 使用**专门为固定翼设计的 PID 控制器**，不是 `AC_AttitudeControl`（那是多旋翼的）：
  - `AP_RollController rollController` — 滚转 → 副翼
  - `AP_PitchController pitchController` — 俯仰 → 升降舵
  - `AP_YawController yawController` — 偏航 → 方向舵（侧滑抑制，协调转弯）
  - `AP_SteerController steerController` — 地面转向
- 核心特性：速度缩放（空速高→舵面减小，空速低→舵面增大）
- `QuadPlane::use_fw_attitude_controllers()` 决定当前用固定翼控制器还是 VTOL 控制器

### Plane 高度控制
- **核心文件**：`ArduPlane/altitude.cpp`
- 做什么：计算高度误差，设定目标爬升率，传递给 TECS

### Plane 导航逻辑
- **核心文件**：`ArduPlane/navigation.cpp`
- 做什么：航点切换判断、距离计算、盘旋逻辑等

---

## Part 3B：VTOL 控制链（QuadPlane 多旋翼侧）

> 这部分和 ArduCopter 几乎一样。QuadPlane 在多旋翼模式下复用了 Copter 的完整控制栈。

### 位置控制 `AC_PosControl`
- **核心文件**：`libraries/AC_AttitudeControl/AC_PosControl.h/cpp`
- 做什么：位置误差（m）→ 速度目标 → 加速度目标 → **倾角目标**（°）
- 四个环路（由外到内）：
  - XY 位置环：位置差 → 目标速度（P）— `AC_P_2D _p_pos_ne_m`
  - XY 速度环：速度差 → 目标加速度（PID）— `AC_PID_2D _pid_vel_ne_m`
  - Z 位置环：高度差 → 目标垂直速度（P）— `AC_P_1D _p_pos_d_m`
  - Z 速度环 → Z 加速度环（PID 级联）— `_pid_vel_d_m` → `_pid_accel_d_m`

### 姿态控制 `AC_AttitudeControl`
- **字段类型**：`AC_AttitudeControl_Multi *attitude_control`
- **实际实例化**：`AC_AttitudeControl_TS`（`quadplane.cpp:822`），继承自 `AC_AttitudeControl_Multi`
- 级联结构：角度误差（P）→ 目标角速度 → 角速度误差（PID + feedforward）→ 力矩

### 电机混控
- motors 字段是 `AP_MotorsMulticopter*`，根据 `Q_FRAME_CLASS` 动态决定实际类型：
  - `QUAD/HEXA/OCTA/Y6/DECA` → `AP_MotorsMatrix`（最常见）
  - `TRI` → `AP_MotorsTri`
  - `TAILSITTER` → `AP_MotorsTailsitter`
  - `SCRIPTING_MATRIX` → `AP_MotorsMatrix_Scripting_Dynamic`
- 将 [油门, Roll, Pitch, Yaw] 通过混控矩阵映射到各升力电机

---

## Part 4：过渡层 — Transition

> 第一性原理：过渡的本质是**推力源的切换** + **控制律的切换**。从"升力电机承担全部重量 + 多旋翼 PID 控制姿态"切换为"机翼产生升力 + TECS/L1 控制轨迹"。中间有一段两者同时作用的"混合区"。

### 过渡抽象基类 `Transition`
- **核心文件**：`ArduPlane/transition.h:21-72`
- 纯虚接口：`update()`、`VTOL_update()`、`complete()`、`restart()`、`force_transition_complete()`
- 两份具体实现：

### SLT_Transition — 独立升力推力型（最常见）
- **核心文件**：`ArduPlane/transition.h:75-133`
- 适用机型：有独立推进电机 + 独立升力电机的垂起（如 4+1 布局）
- 过渡状态：`AIRSPEED_WAIT`（等空速）→ `TIMER`（定时过渡）→ `DONE`（完成）
- 执行文件：`ArduPlane/quadplane.cpp` 中的过渡逻辑

### Tailsitter_Transition — 尾座式
- **核心文件**：`ArduPlane/tailsitter.h`
- 继承自 `Transition` 基类（不继承 SLT_Transition）
- 特点：整架飞机从垂直姿态旋转 90° 变为水平，姿态控制逻辑完全不同
- 过渡状态：`ANGLE_WAIT_FW`（等俯仰角达到前飞角度）→ `ANGLE_WAIT_VTOL` → `DONE`（完成）
- 没有独立的升力电机，同一组电机既是升力电机也是推进电机

### Tiltrotor_Transition — 倾斜旋翼
- **核心文件**：`ArduPlane/tiltrotor.h`
- 继承自 `SLT_Transition`，增加偏航控制在过渡中保持方向
- 倾斜类型：`CONTINUOUS`（伺服连续变倾角）、`BINARY`（仅垂直/水平两位置）、`VECTORED_YAW`（倾斜电机矢量控制偏航）、`BICOPTER`（双桨构型）

### 三种 VTOL 机械构型
| 类型 | `ThrustType` 枚举 | 特点 |
|------|-------------------|------|
| SLT | `SLT=0` | 独立推进电机 + 独立升力电机（如 4+1） |
| Tailsitter | `TAILSITTER` | 机身垂直起降，飞行中旋转 90° |
| Tiltrotor | `TILTROTOR` | 电机可倾斜改变推力方向 |

---

## Part 5：QuadPlane 主类 — VTOL 的核心调度中心

- **核心文件**：`ArduPlane/quadplane.h/cpp`（~7000+ 行的核心文件）
- 关键成员：
  - `AP_MotorsMulticopter *motors` — VTOL 升力电机
  - `AC_AttitudeControl_Multi *attitude_control` — VTOL 姿态控制
  - `AC_PosControl *pos_control` — VTOL 位置控制
  - `AC_WPNav *wp_nav` — 航点导航（VTOL 侧）
  - `AC_Loiter *loiter_nav` — 悬停导航（VTOL 侧）
  - `Transition *transition` — 当前过渡状态机
- 其他关键成员：
  - `Tailsitter tailsitter` — 尾座式控制（值成员，非指针）
  - `Tiltrotor tiltrotor` — 倾斜旋翼控制（值成员，非指针）
  - `VTOL_Assist assist` — **VTOL 辅助系统**（固定翼模式中自动启用多旋翼防止失速）
  - `AC_WeatherVane *weathervane` — 迎风偏航（自动转至迎风方向）
  - `AP_AHRS_View *ahrs_view` — 姿态旋转视图
- 核心方法：
  - `update()` — 每个主循环调用，调度一切。分支逻辑：`in_vtol_mode()` ? VTOL 侧 : 固定翼侧
  - `vtol_position_controller()` — 9 状态 VTOL 着陆序列：APPROACH → AIRBRAKE → POSITION1 → POSITION2 → LAND_DESCEND → LAND_FINAL → LAND_COMPLETE
  - `control_auto()` — 自动任务中的 VTOL 控制
  - `hold_hover()` / `hold_stabilize()` — 悬停/增稳（`control_hover()` 声明了但未实现，是死代码）
  - `in_assisted_flight()` — 是否在辅助飞行模式
  - `should_disable_TECS()` — VTOL 模式下禁用 TECS

---

## Part 5B：VTOL 辅助系统 — 固定翼模式的"安全网"

- **核心文件**：`ArduPlane/VTOL_Assist.h`
- 做什么：在固定翼模式下自动启用多旋翼电机，防止失速或姿态失控
- 触发条件：
  - 空速过低（< `ASSIST_SPEED` 阈值）
  - 姿态偏差过大（> `ASSIST_ANGLE` 阈值）
  - 高度损失过多
  - 也可通过 RC 辅助开关强制启用（`FORCE_ENABLED`）
- `VTOL_Assist::should_assist()` 被 `SLT_Transition::update()` 和 `Tailsitter_Transition::update()` 调用
- 这是飞行安全的关键系统，固定翼失速时多旋翼自动介入

---

## Part 6：固定翼飞行模式

> 所有模式直接继承 `Mode` 基类（`ArduPlane/mode.h:29`），不存在模式之间的代码继承。

### 6.1 手动/半自动模式

| 模式 | 编号 | 说明 |
|------|------|------|
| `MANUAL` | 0 | 完全手动，摇杆直通舵面，无自稳 |
| `FBWA` | 5 | Fly-by-Wire A：横滚角对应摇杆角度（有倾角限制），俯仰对应摇杆，油门手动，**最常用的手动增强模式** |
| `FBWB` | 6 | Fly-by-Wire B：类似 FBWA 但油门自动控制空速，俯仰控制高度 |
| `CRUISE` | 7 | 巡航：保持当前航向/高度/空速，类似固定翼版的 Loiter |
| `STABILIZE` | 2 | 自稳：类似 FBWA 但手动油门 |
| `ACRO` | 4 | 特技：摇杆对应角速率，无倾角限制 |

### 6.2 自动模式

| 模式 | 编号 | 说明 |
|------|------|------|
| `AUTO` | 10 | 全自动任务：起飞→航点→降落 |
| `RTL` | 11 | 返航：自动规划路径回家并盘旋/降落 |
| `LOITER` | 12 | 在当前位置盘旋 |
| `TAKEOFF` | 13 | 自动起飞 |
| `GUIDED` | 15 | 地面站实时控制 |

### 6.3 固定翼模式的核心循环

以 FBWA 为例，`run()` 做的事：
1. 读取遥控器摇杆 → 转换为目标滚转角（倾斜角限制内）和目标俯仰角
2. `L1_Control` → 计算所需的滚转修正（如果有导航目标）
3. `TECS` → 计算所需的油门和俯仰（如果有高度/速度目标）
4. `Attitude.cpp` → 将目标滚转/俯仰转换为副翼/升降舵的 PWM 值
5. `set_servos()` → 输出到硬件

---

## Part 7：VTOL 飞行模式（Q 模式）

> 与固定翼模式不同，**Q 模式之间存在代码委托**。`QStabilize::update()` 是所有 Q 模式的更新基础，其他 Q 模式都委托给它。`QLand::run()` 使用 `QLoiter::run()`。

| 模式 | 编号 | 说明 | 委托关系 |
|------|------|------|---------|
| `QSTABILIZE` | 17 | VTOL 自稳：手动角度，手动油门（多旋翼模式） | 所有 Q 模式委托其 `update()` |
| `QHOVER` | 18 | VTOL 悬停：定高 + 水平定点悬停 | 委托 `QStabilize::update()` + `hold_hover()` |
| `QLOITER` | 19 | VTOL 定点：悬停 + 摇杆可移动位置 | 委托 `QStabilize::update()` + `AC_Loiter` |
| `QLAND` | 20 | VTOL 降落：自动垂直降落 | 委托 `QLoiter::run()` + 着陆状态机 |
| `QRTL` | 21 | VTOL 返航：自动返航并用 VTOL 方式降落 | 委托 `vtol_position_controller()` |
| `QACRO` | 23 | VTOL 特技：手动角速率模式 | 委托 `ModeAcro::run()`（固定翼 Acro 的逻辑） |

> 还有 `QAUTOTUNE`(22) 自动调参（委托 `QStabilize::update()`）、`LOITER_ALT_QLAND`(25) 先固定翼盘旋到低高度再用 VTOL 降落。

### Q 模式的控制流

以 QHOVER 为例，`run()` 做的事：
1. `AC_PosControl` → 当前位置 vs 目标位置 → 目标速度 → 目标倾角
2. `AC_AttitudeControl_TS` → 目标倾角 → 力矩（PID + feedforward）
3. `AP_MotorsMatrix`（或其他类型）→ 力矩 → 各电机 PWM
4. `quadplane.update()` → 协调固定翼舵面（辅助稳定）+ 过渡检查

---

## Part 8：执行层（共享）

### 舵机和电机输出 `SRV_Channel`
- **核心文件**：`libraries/SRV_Channel/SRV_Channel.h/cpp`、`SRV_Channels.cpp`
- 固定翼输出：副翼、升降舵、方向舵、襟翼、推进电机/油门
- VTOL 输出：4 个升力电机、副翼、升降舵、方向舵、推进电机
- ~190 种通道功能映射

### 固定翼舵面混控 `AP_Motors`
- 固定翼侧用 `AP_Motors` 处理舵面混控（不同于多旋翼矩阵混控）
- `set_servos()` 在 `ArduPlane/Plane.cpp:66` 调用，包含 `quadplane.update()` 和舵机输出

---

## Part 9：横切面

### 参数系统 `AP_Param`
- 核心文件：`libraries/AP_Param/AP_Param.h/cpp`
- 参数通过 `AP_GROUPINFO` 宏注册，索引不可变
- Plane/QuadPlane 关键参数前缀：
  - `Q_*`：QuadPlane 参数（Q_ENABLE、Q_FRAME_CLASS、Q_FRAME_TYPE 等）
  - `TECS_*`：TECS 参数
  - `L1_*`：L1 导航参数
  - `ARSPD_*`：空速计参数
  - `PTCH_*`、`RLL_*`：俯仰/滚转 PID 参数

### 日志系统 `AP_Logger`
- 核心文件：`libraries/AP_Logger/AP_Logger.h/cpp`、`LogStructure.h`
- 后端：SD 卡文件、MAVLink 远程、板载 Flash
- 固定翼关键日志：TECS、L1、ARSP、AETR（舵面输出）
- QuadPlane 关键日志：QTUN（Q_Control_Tuning）、Q_TRANSITION
- **SITL → 看日志 = 最快调试路径**

---

## 学习路径（固定翼/垂起视角）

```
第一轮：传感器 + 执行层
  → GPS/IMU/Baro/Airspeed 各读什么
  → SRV_Channel 怎么输出到舵机和电机
  → 理解数据从哪来、到哪去

第二轮：状态估计
  → AHRS 怎么从传感器融合出姿态
  → EKF 怎么估计位置/速度/风速
  → 特别注意空速和风速的融合（固定翼独有）

第三轮：固定翼控制
  → TECS 怎么协调油门和俯仰（能量管理的直觉）
  → L1 怎么实现横向导航（参考点法）
  → FBWA 模式从摇杆到舵面的完整链路

第四轮：VTOL 控制
  → QHOVER 模式的控制链路（和 Copter 一样，但加了 QuadPlane 调度）
  → AP_MotorsMatrix 混控

第五轮：过渡
  → SLT_Transition 的状态机
  → 推力混合（throttle_mix）怎么在两个系统间切换
  → Tailsitter 和 Tiltrotor 的区别

横切面：参数(Q_*/TECS_*/L1_*)和日志(QTUN/Q_TRANSITION)贯穿全程
```

**费曼检验**：每轮完成后，不看代码画出该环节数据流图（标注物理单位和频率）。
