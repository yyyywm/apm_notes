# Tailsitter 参数解读

> 机型：**三角翼 Tailsitter VTOL（无矢量推力）** — 双主推电机差速 yaw + 双 elevon 控制面
> 实际接线：仅 SERVO5/6（Elevon）+ SERVO9/10（ThrottleLeft/Right）有效；SERVO3/4 虽分配了 TiltMotor 功能但**未接线**
> 参数文件：`/Users/ywm/Desktop/Flight/eXploraVTOL/parameters/Ardupilot/2022-10-17_Pixhawk1.param`
> 解读依据：ArduPilot 源码（`ArduPlane/tailsitter.cpp`、`ArduPlane/quadplane.cpp` 等）中的 `@Values`、`@Range`、`@Bitmask`、`@Units` 注释 + 官方 SRV_Channel function 编号表

---

## 一、机型与帧类型（核心识别）

| 参数                 | 当前值    | 可选值 / 范围                                                                                                                                                      | 当前值含义                  |
| ------------------ | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------- |
| `Q_ENABLE`         | 1      | 0:禁用 / 1:启用 / 2:启用VTOL自动模式                                                                                                                                    | **启用 QuadPlane**       |
| `Q_FRAME_CLASS`    | **10** | 0:未定义 / 1:Quad / 2:Hexa / 3:Octa / 4:OctaQuad / 5:Y6 / 7:Tri / **10:Single/Dual(Tailsitter)** / 12:DodecaHexa / 14:Deca / 15:Scripting / 17:Dynamic Scripting | **Tailsitter 单/双电机帧**  |
| `Q_FRAME_TYPE`     | 1      | 0:+ / 1:X / 2:V / 3:H / 4:V-Tail / 5:A-Tail / 10:Y6B / 11:Y6F / 12:BFX / 13:DJIX / 14:CWX / 15:I / 18:BFXR / 19:Y4                                            | X 型（双电机时基本忽略）          |
| `Q_TAILSIT_ENABLE` | 1      | 0:禁用 / 1:启用 / **2:无控制面强制启用**（强制 Qassist + 全程 airmode）                                                                                                         | **正常启用 tailsitter 控制** |
| `Q_TILT_ENABLE`    | 0      | 0:禁用 / 1:启用                                                                                                                                                   | 不使用倾转电机（**确认无矢量推力**）   |
| `AHRS_ORIENTATION` | 2      | 0:None / 1:Yaw45 / **2:Yaw90** / 3:Yaw135 / 4:Yaw180 / ... / 38:Custom                                                                                        | 飞控水平安装+机头朝右 90°        |

---

## 二、Tailsitter 专属参数 Q_TAILSIT_*

| 参数                 | 当前值             | 范围/可选值                                                                   | 默认值 | 当前值含义                                                           |
| ------------------ | --------------- | ------------------------------------------------------------------------ | --- | --------------------------------------------------------------- |
| `Q_TAILSIT_ANGLE`  | 60              | 5–80°                                                                    | 45° | **VTOL→FW** 过渡完成俯冲到 60°                                         |
| `Q_TAILSIT_ANG_VT` | 60              | 5–80°，0=用 ANGLE                                                          | 0   | **FW→VTOL** 拉起到 60° 触发完成                                        |
| `Q_TAILSIT_RAT_FW` | 60              | 10–500°/s                                                                | 50  | 前飞过渡的 pitch 速率 60°/s                                            |
| `Q_TAILSIT_RAT_VT` | 60              | 10–500°/s                                                                | 50  | 回 VTOL 拉起的 pitch 速率 60°/s                                       |
| `Q_TAILSIT_THR_VT` | 60              | -1–100%（-1=用悬停油门）                                                        | -1  | 回 VTOL 期间油门固定 60%                                               |
| `Q_TAILSIT_INPUT`  | 0               | Bitmask: bit0=PlaneMode / bit1=BodyFrameRoll                             | 0   | **多旋翼视角输入**（roll 摇杆=机体 roll）                                    |
| `Q_TAILSIT_MOTMX`  | 0               | Bitmask: bit0~11 = motor 1~12（0=非Copter式）                                | 0   | **Single/Dual 双电机布局**（不用 Copter motor matrix）                   |
| `Q_TAILSIT_GSCMSK` | **11** (0b1011) | Bitmask: bit0:Throttle / bit1:ATT_THR / bit2:Disk Theory / bit3:Altitude | 1   | **Throttle + ATT_THR + Altitude** 三种增益缩放叠加（bit2 Disk Theory 关闭） |
| `Q_TAILSIT_GSCMIN` | 0.4             | 0.1–1                                                                    | 0.4 | 高速时舵面增益最小缩到 0.4×                                                |
| `Q_TAILSIT_GSCMAX` | 4               | 1–5                                                                      | 2   | 低速时舵面增益最大放大 4×（**比默认更激进**）                                      |
| `Q_TAILSIT_DSKLD`  | 18              | 0–50 kg/m²                                                               | 0   | 桨盘载荷 18 kg/m²（GSCMSK 没启用 bit2，**实际不生效**）                        |
| `Q_TAILSIT_MIN_VO` | 4               | 0–15 m/s                                                                 | 0   | 最小桨后流速 4 m/s（配合 Disk Theory，**当前不生效**）                          |
| `Q_TAILSIT_RLL_MX` | 30              | 0–80°（0=用 Q_A_ANGLE_MAX）                                                 | 0   | 悬停最大 roll 30°                                                   |
| `Q_TAILSIT_VFGAIN` | 0               | 0–1                                                                      | 0   | 矢量前飞增益（无矢量=0）                                                   |
| `Q_TAILSIT_VHGAIN` | 0.5             | 0–1                                                                      | 0.5 | 矢量悬停增益（无矢量推力机型应改为 0，详见第四章建议）                                    |
| `Q_TAILSIT_VHPOW`  | 2.5             | 0–4                                                                      | 2.5 | 矢量悬停指数（**不生效**）                                                 |
| `Q_TAILSIT_VT_R_P` | 1               | 0–2                                                                      | 1   | VTOL roll：PID 输出→舵面缩放=1（**双轴共控才用，对纯舵面/纯电机布局基本不影响**）             |
| `Q_TAILSIT_VT_P_P` | 1               | 0–2                                                                      | 1   | VTOL pitch 同上                                                   |
| `Q_TAILSIT_VT_Y_P` | 1               | 0–2                                                                      | 1   | VTOL yaw 同上                                                     |

> ⚠️ `Q_TAILSIT_GSCMSK=11` 表示同时开启 **节流缩放、姿态-油门混控、海拔(空气密度)修正**；但 bit2（Disk Theory）未开，所以 `DSKLD` 与 `MIN_VO` 仅作占位。

---

## 三、过渡逻辑

| 参数                 | 当前值  | 范围                         | 当前值含义                          |
| ------------------ | ---- | -------------------------- | ------------------------------ |
| `Q_TRANSITION_MS`  | 2000 | 500–30000 ms               | 前飞过渡总时长 2 s                    |
| `Q_BACKTRANS_MS`   | 3000 | —                          | 回 VTOL 过渡时长 3 s                |
| `Q_TRAN_PIT_MAX`   | 3    | 0–30°                      | 过渡中 pitch 限制因子 3               |
| `Q_TRANS_DECEL`    | 4    | — m/s²                     | 回 VTOL 时减速度 4 m/s²             |
| `Q_ASSIST_SPEED`   | 0    | -1=禁用 / 0=触发预解锁失败 / >0:m/s | **0 表示未配置**（建议改为 -1 或 巡航最小速-3） |
| `Q_ASSIST_ANGLE`   | 30   | 0–90°                      | 姿态偏离 >30° 触发 VTOL 辅助           |
| `Q_ASSIST_DELAY`   | 0.5  | s                          | 辅助触发延时 0.5 s                   |
| `Q_ASSIST_ALT`     | 0    | m                          | 高度低于此值才辅助（0=禁用）                |
| `Q_TRANS_FAIL`     | 0    | s                          | 过渡超时触发故障保护（0=禁用）               |
| `Q_TRANS_FAIL_ACT` | 0    | 0:QLAND / 1:QRTL           | QLAND                          |

---

## 四、舵机/电机功能映射 SERVOx_FUNCTION

ArduPilot SRV_Channel function 官方编号（关键值）：

| 编号    | 功能                  |
| ----- | ------------------- |
| 70    | Throttle            |
| 73    | **ThrottleLeft**    |
| 74    | **ThrottleRight**   |
| 75    | TiltMotorFrontLeft  |
| 76    | TiltMotorFrontRight |
| 77    | **ElevonLeft**      |
| 78    | **ElevonRight**     |
| 79    | VTailLeft           |
| 80    | VTailRight          |
| 19    | Elevator            |
| 21    | Rudder              |
| 4     | Aileron             |
| 33–40 | Motor1–Motor8       |

参数文件实际映射 + 接线情况：

| 通道      | FUNCTION 当前值 | 含义                        | 实际接线            |
| ------- | ------------ | ------------------------- | --------------- |
| SERVO3  | 76           | TiltMotorFrontRight       | ❌ **未接线**（参数遗留） |
| SERVO4  | 75           | TiltMotorFrontLeft        | ❌ **未接线**（参数遗留） |
| SERVO5  | **77**       | **ElevonLeft**（左升降副翼）     | ✅ 已接线           |
| SERVO6  | **78**       | **ElevonRight**（右升降副翼，反向） | ✅ 已接线           |
| SERVO9  | **73**       | **ThrottleLeft**（左主推电机）   | ✅ 已接线           |
| SERVO10 | **74**       | **ThrottleRight**（右主推电机）  | ✅ 已接线           |

**实际生效的控制布局**（无矢量推力三角翼 tailsitter）：

```
        [机头]
          ▲
          │
   ┌──────┴──────┐
   │             │
   │  Δ 三角翼    │
   │             │
[左elevon]   [右elevon]   ← 控制面：pitch 同向 / roll 差动
   │             │
   ●             ●         ← SERVO9/10：左右主推电机
  左推           右推         油门同向 = 推力；差速 = yaw
```

**控制混控分工**：

| 轴            | 主控源                                          | 备注                                              |
| ------------ | -------------------------------------------- | ----------------------------------------------- |
| **Pitch**    | 双 elevon 同向偏转                                | VTOL 时低速依赖 GSCMAX 增益放大                          |
| **Roll**     | 双 elevon 差动偏转 + 双电机差速油门（VTOL 时由 mixing 矩阵贡献） | 低速时电机贡献为主，高速时 elevon 为主                         |
| **Yaw**      | **双电机差速油门**（无方向舵）                            | VTOL：核心 yaw 来源；FW：靠 `RUDD_DT_GAIN=10` 的差速油门 yaw |
| **Throttle** | 双电机同向                                        | —                                               |

> ⚠️ **建议清理 SERVO3/4 残留配置**
> 虽然硬件上未接线，但 `tailsitter.cpp:220-223` 的源码逻辑会因 `k_tiltMotorLeft/Right` 被分配（function 75/76 已设置）+ `Q_TAILSIT_VHGAIN=0.5` ≠ 0 而判定 `_is_vectored = true`，激活矢量分支代码：
> 
> ```cpp
> _is_vectored = (frame_class == TAILSITTER) &&
>                (!is_zero(vectored_hover_gain) &&
>                 (k_tiltMotorLeft assigned || k_tiltMotorRight assigned));
> ```
> 
> 即使输出端无电调，飞控内部仍会按矢量逻辑运算（可能产生未使用的输出值，浪费 CPU 且增加调试复杂度）。
> 
> **推荐二选一彻底关闭矢量分支**：
> 
> - **方案 A（推荐）**：`SERVO3_FUNCTION = 0`、`SERVO4_FUNCTION = 0`（取消 TiltMotor 分配）
> - **方案 B**：`Q_TAILSIT_VHGAIN = 0`（保留 SERVO 功能但禁用矢量增益）
> 
> 任一方案都会让 `_is_vectored = false`，进入纯舵面 tailsitter 控制路径，与你的实际硬件一致。

---

## 五、VTOL 姿态/速率 PID（Q_A_*）

| 参数                        | 当前值                | 范围/含义             |
| ------------------------- | ------------------ | ----------------- |
| `Q_A_ANG_RLL_P`           | 4                  | 0–12              |
| `Q_A_ANG_PIT_P`           | 3                  | 0–12              |
| `Q_A_ANG_YAW_P`           | 3.5                | 0–12              |
| `Q_A_RAT_RLL_P/I/D`       | 0.2 / 0.15 / 0.02  | 速率内环              |
| `Q_A_RAT_PIT_P/I/D`       | 0.17 / 0.2 / 0.005 | —                 |
| `Q_A_RAT_YAW_P/I/D`       | 0.4 / 0.5 / 0.01   | yaw I 较大（差速电机响应慢） |
| `Q_A_RAT_*_FF`            | 0.18~0.2           | 前馈系数              |
| `Q_A_RATE_R/P/Y_MAX`      | 75                 | 0–1080 °/s        |
| `Q_A_ACCEL_R/P/Y_MAX`     | 25k/30k/27k        | 0–180000 cdeg/s²  |
| `Q_A_THR_MIX_MIN/MAX/MAN` | 0.1/0.9/0.9        | 0.1–2             |
| `Q_A_RATE_FF_ENAB`        | 1                  | 0/1               |
| `Q_A_ANGLE_BOOST`         | 0                  | 0/1               |
| `Q_A_INPUT_TC`            | 0.2                | 0–1 s             |
| `Q_A_SLEW_YAW`            | 6000               | cd/s              |

---

## 六、VTOL 电机参数（Q_M_*）

| 参数                    | 当前值       | 可选/范围                                                                                                                              | 含义               |
| --------------------- | --------- | ---------------------------------------------------------------------------------------------------------------------------------- | ---------------- |
| `Q_M_PWM_MIN/MAX`     | 1000/2000 | 800–2200                                                                                                                           | ESC PWM          |
| `Q_M_PWM_TYPE`        | **4**     | 0:Normal / 1:OneShot / 2:OneShot125 / 3:Brushed / **4:DShot150** / 5:DShot300 / 6:DShot600 / 7:DShot1200 / 8:PWMRange / 9:PWMAngle | DShot150 数字 ESC  |
| `Q_M_SPIN_MIN`        | 0.15      | 0–0.3                                                                                                                              | 最小有效油门 15%       |
| `Q_M_SPIN_MAX`        | 0.95      | 0.9–1                                                                                                                              | 最大油门 95%         |
| `Q_M_SPIN_ARM`        | 0         | 0–0.3                                                                                                                              | 解锁后不空转           |
| `Q_M_THST_HOVER`      | 0.235     | 0.125–0.6875                                                                                                                       | 学习到的悬停油门 23.5%   |
| `Q_M_THST_EXPO`       | 0.65      | -1 to 1                                                                                                                            | 推力指数补偿（电机推力非线性）  |
| `Q_M_HOVER_LEARN`     | 2         | 0:Disable / 1:Learn / **2:Learn+Save**                                                                                             | 自学习并保存           |
| `Q_M_YAW_HEADROOM`    | 200       | 0–500                                                                                                                              | yaw 控制 PWM 余量    |
| `Q_M_SPOOL_TIME`      | 0.25      | 0–2 s                                                                                                                              | spool up 时间      |
| `Q_M_SAFE_DISARM`     | 0         | 0/1                                                                                                                                | 解锁安全检查           |
| `Q_M_SLEW_UP/DN_TIME` | 0/0       | s                                                                                                                                  | 油门 slew 限速（0=禁用） |
| `Q_M_BOOST_SCALE`     | 0         | —                                                                                                                                  | boost 油门（不使用）    |

---

## 七、VTOL 位置/速度（Q_P_*, Q_WP_*, Q_LOIT_*）

| 参数                            | 当前值        | 含义              |
| ----------------------------- | ---------- | --------------- |
| `Q_P_POSXY_P`                 | 0.5        | 水平位置环 P         |
| `Q_P_POSZ_P`                  | 1          | 垂直位置环 P         |
| `Q_P_VELXY_P/I/D`             | 1/0.5/0.25 | 水平速度 PID        |
| `Q_P_VELZ_P`                  | 5          | 垂直速度 P          |
| `Q_P_ACCZ_P/I/IMAX`           | 0.3/1/800  | Z 加速度 PI        |
| `Q_P_JERK_XY/Z`               | 2/5        | m/s³            |
| `Q_VELZ_MAX` / `_DN`          | 500/150    | cm/s 上升/下降      |
| `Q_ACCEL_Z`                   | 250        | cm/s²           |
| `Q_ANGLE_MAX`                 | 2000       | cd = 20° 倾角限幅   |
| `Q_LOIT_SPEED`                | 300        | cm/s 悬停模式最大水平速  |
| `Q_LOIT_ANG_MAX`              | 0          | 0=用 Q_ANGLE_MAX |
| `Q_LOIT_BRK_ACCEL/JERK/DELAY` | 50/250/1   | 刹车              |
| `Q_WP_SPEED`                  | 500        | cm/s 航点速度       |
| `Q_WP_SPEED_UP/DN`            | 300/150    | cm/s            |
| `Q_WP_RADIUS`                 | 200        | cm 抵达半径         |
| `Q_WP_ACCEL/_Z`               | 100/100    | cm/s²           |
| `Q_WP_JERK`                   | 1          | m/s³            |
| `Q_WP_TER_MARGIN`             | 10         | m 地形跟随余量        |

---

## 八、风向标 Q_WVANE_*

| 参数                 | 当前值   | 可选                                            | 含义        |
| ------------------ | ----- | --------------------------------------------- | --------- |
| `Q_WVANE_ENABLE`   | **3** | 0:禁用 / 1:Loiter / 2:Loiter+QAUTO / **3:始终启用** | 全程开启风向标对齐 |
| `Q_WVANE_GAIN`     | 2     | 0.5–4                                         | 增益        |
| `Q_WVANE_ANG_MIN`  | 1     | ° 触发最小角                                       |           |
| `Q_WVANE_LAND`     | 3     | 着陆时增益                                         |           |
| `Q_WVANE_HGT_MIN`  | 0     | m 触发最低高度                                      |           |
| `Q_WVANE_TAKEOFF`  | 0     | 起飞不用                                          |           |
| `Q_WVANE_VELZ_MAX` | 0     | 0=禁用                                          |           |
| `Q_WVANE_SPD_MAX`  | 0     | 0=禁用                                          |           |

---

## 九、Q_RTL / 其他 VTOL

| 参数                           | 当前值         | 含义                                                |
| ---------------------------- | ----------- | ------------------------------------------------- |
| `Q_RTL_ALT`                  | 20          | m QRTL 高度                                         |
| `Q_RTL_MODE`                 | 1           | 0:禁用 / **1:启用** / 2:VTOL approach / 3:QRTL Always |
| `Q_LAND_FINAL_ALT`           | 10          | m 最终着陆高度阈值                                        |
| `Q_LAND_SPEED`               | 50          | cm/s 着陆速度                                         |
| `Q_LAND_ALTCHG`              | 0.2         | m 触地检测高度变化                                        |
| `Q_LAND_ICE_CUT`             | 1           | 着陆切发动机                                            |
| `Q_GUIDED_MODE`              | 0           | 0:禁用 / 1:启用                                       |
| `Q_TKOFF_ARSP_LIM`           | 0           | 起飞最大空速（0=禁用）                                      |
| `Q_TKOFF_FAIL_SCL`           | 0           | 起飞失败缩放                                            |
| `Q_OPTIONS`                  | 0           | bitmask（0=无特殊选项）                                  |
| `Q_FW_LND_APR_RAD`           | 0           | m VTOL approach 半径                                |
| `Q_VFWD_GAIN`                | 0           | 前向推力辅助（0=禁用）                                      |
| `Q_VFWD_ALT`                 | 0           | m 触发高度                                            |
| `Q_PLT_Y_RATE`               | 150         | °/s 飞手 yaw 输入限速                                   |
| `Q_PLT_Y_EXPO`               | 0.25        | yaw 摇杆 expo                                       |
| `Q_PLT_Y_RATE_TC`            | 0.25        | s yaw 时间常数                                        |
| `Q_THROTTLE_EXPO`            | 0.2         | 油门 expo                                           |
| `Q_NAVALT_MIN`               | 0           | m 起降最小导航高度                                        |
| `Q_TRIM_PITCH`               | 0           | ° 悬停 pitch 配平                                     |
| `Q_AUTOTUNE_AGGR/AXES/MIN_D` | 0.1/7/0.001 | 自动调参参数                                            |
| `Q_ACRO_RLL/PIT/YAW_RATE`    | 360/180/90  | °/s ACRO 速率                                       |

---

## 十、固定翼姿态 PID（巡航）

| 参数                          | 当前值              | 含义                                                   |
| --------------------------- | ---------------- | ---------------------------------------------------- |
| `RLL_RATE_P/I/D`            | 0.17/0.13/0.0022 | Roll 速率 PID                                          |
| `RLL_RATE_FF`               | 0.092            | 前馈                                                   |
| `RLL2SRV_RMAX`              | 90               | °/s roll 最大角速度                                       |
| `RLL2SRV_TCONST`            | 0.3              | s 时间常数                                               |
| `PTCH_RATE_P/I/D`           | 0.06/0.1/0.003   | Pitch 速率 PID                                         |
| `PTCH2SRV_TCONST`           | 0.45             | s                                                    |
| `PTCH2SRV_RMAX_UP/DN`       | 90/90            | °/s                                                  |
| `PTCH2SRV_RLL`              | 1                | roll→pitch 补偿                                        |
| `YAW2SRV_DAMP/INT/RLL/SLIP` | 0/0/1/0          | 航向阻尼/积分/roll补偿/侧滑                                    |
| `KFF_RDDRMIX`               | 0.02             | 副翼→方向舵交联（无方向舵基本无效）                                   |
| `RUDD_DT_GAIN`              | **10**           | 0–100                                                |
| `LIM_ROLL_CD`               | 5000             | cd                                                   |
| `LIM_PITCH_MAX`             | 4000             | cd                                                   |
| `LIM_PITCH_MIN`             | -3000            | cd                                                   |
| `STAB_PITCH_DOWN`           | 2                | ° 低速 pitch 下压                                        |
| `LEVEL_ROLL_LIMIT`          | 5                | ° 起降阶段 roll 限幅                                       |
| `STALL_PREVENTION`          | 1                | 0/1 失速保护                                             |
| `USE_REV_THRUST`            | 2                | bitmask 反推使用场景                                       |
| `STICK_MIXING`              | 1                | 0:禁用 / 1:FBWMixing / 2:DirectMixing                  |
| `MIXING_GAIN`               | 0.7              | 0.5–1.2 elevon 混控增益（tailsitter 默认 1.0，**这里被改成 0.7**） |

---

## 十一、空速/导航/TECS

| 参数                  | 当前值         | 含义                                                              |
| ------------------- | ----------- | --------------------------------------------------------------- |
| `ARSPD_FBW_MIN/MAX` | 12/20       | m/s 巡航空速范围                                                      |
| `ARSPD_USE`         | 1           | 启用                                                              |
| `ARSPD_TYPE`        | 2           | 0:None / 1:I2C-MS4525 / **2:Analog** / 3:Pitot / 4:MS5525 / ... |
| `ARSPD_PIN`         | 15          | 模拟引脚                                                            |
| `ARSPD_RATIO`       | 2.527       | 校准                                                              |
| `ARSPD_OPTIONS`     | 11 (0b1011) | bitmask 多重选项启用                                                  |
| `ARSPD_AUTOCAL`     | 0           | 不自动校准                                                           |
| `TRIM_ARSPD_CM`     | 1600        | cm/s 巡航目标 16 m/s                                                |
| `SCALING_SPEED`     | 15          | m/s 舵效缩放参考速度                                                    |
| `MIN_GNDSPD_CM`     | 0           | 最小地速（0=禁用）                                                      |
| `NAVL1_PERIOD`      | 21          | s 越大越平滑                                                         |
| `NAVL1_DAMPING`     | 0.75        | 0.6–1 阻尼                                                        |
| `NAVL1_XTRACK_I`    | 0.02        | 横向偏离积分                                                          |
| `TECS_CLMB_MAX`     | 5           | m/s 最大爬升                                                        |
| `TECS_SINK_MAX/MIN` | 5/2         | m/s                                                             |
| `TECS_TIME_CONST`   | 5           | s                                                               |
| `TECS_THR_DAMP`     | 0.5         |                                                                 |
| `TECS_INTEG_GAIN`   | 0.1         |                                                                 |
| `TECS_SPDWEIGHT`    | 1           | 0:全用 pitch / 1:平衡 / 2:全用油门                                      |
| `TECS_VERT_ACC`     | 7           | m/s² 垂直加速度                                                      |
| `TECS_SYNAIRSPEED`  | 0           | 不用合成空速                                                          |
| `THR_MAX/MIN`       | 90/0        | % 油门限幅                                                          |
| `THR_SLEWRATE`      | 100         | %/s                                                             |
| `TRIM_THROTTLE`     | 30          | % 巡航油门                                                          |

---

## 十二、飞行模式

`FLTMODE_CH=8`（用 RC8 切换）

| 参数       | 当前值 | 模式（Plane mode 编号）      |
| -------- | --- | ---------------------- |
| FLTMODE1 | 7   | **CRUISE** 巡航          |
| FLTMODE2 | 19  | **QRTL** VTOL 返航       |
| FLTMODE3 | 5   | **FBWA** 增稳辅助          |
| FLTMODE4 | 17  | **QSTABILIZE** VTOL 自稳 |
| FLTMODE5 | 12  | **AUTO** 自动任务          |
| FLTMODE6 | 0   | **MANUAL** 手动          |

Plane 模式编号速查：0=MANUAL, 1=CIRCLE, 2=STABILIZE, 5=FBWA, 6=FBWB, 7=CRUISE, 10=AUTO（旧）, 11=RTL, 12=AUTO, 15=GUIDED, 17=QSTABILIZE, 18=QHOVER, 19=QLOITER/QRTL（按版本）, 22=QLAND, 24=QAUTOTUNE, 25=QACRO

---

## 十三、传感器 / EKF / 校准

| 参数                 | 当前值  | 含义                    |
| ------------------ | ---- | --------------------- |
| `EK3_ENABLE`       | 1    | 启用 EKF3               |
| `EK2_ENABLE`       | 0    | 关闭 EKF2               |
| `EK3_IMU_MASK`     | 3    | bit0+bit1 = IMU1+IMU2 |
| `AHRS_EKF_TYPE`    | 3    | 用 EKF3                |
| `INS_USE/USE2`     | 1/1  | 双 IMU                 |
| `INS_FAST_SAMPLE`  | 1    | 启用 IMU 快采             |
| `COMPASS_USE/USE2` | 1/1  | 双罗盘                   |
| `COMPASS_ORIENT`   | 4    | Yaw180（外置）            |
| `BATT_MONITOR`     | 4    | **Analog V+I**        |
| `BATT_LOW_VOLT`    | 18.6 | V → 6S 低压             |
| `BATT_FS_LOW_ACT`  | 4    | **RTL**               |
| `RNGFND1_TYPE`     | 20   | benewake TFmini/类     |
| `GPS_TYPE`         | 1    | uBlox                 |
| `BRD_TYPE`         | 2    | Pixhawk1              |

---

## 十四、需要重点核实/清理的配置点

1. **SERVO3/4 分配了 TiltMotorFrontLeft/Right (function 75/76) 但实际未接线**
   - 配合 `Q_TAILSIT_VHGAIN=0.5`，源码 `tailsitter.cpp:220-223` 的 `_is_vectored` 判定为 true，会进入矢量分支运算（虽无输出但浪费 CPU、影响日志可读性）
   - **推荐清理**：`SERVO3_FUNCTION=0` + `SERVO4_FUNCTION=0`，或将 `Q_TAILSIT_VHGAIN=0`
2. **`Q_TILT_ENABLE=0`** 与 SERVO3/4 的 TiltMotor 分配字段上不一致：`Q_TILT_ENABLE` 控制 Plane 的 Tiltrotor 模块（独立于 tailsitter 矢量逻辑），但 SERVO 功能仍会触发 tailsitter 内部的 `_is_vectored` 识别。建议按上一条彻底清理
3. **`Q_ASSIST_SPEED=0`** 会导致预解锁警告，建议设为 `-1`（禁用）或正值（如 9 m/s，即 `ARSPD_FBW_MIN - 3`）
4. **`Q_TAILSIT_GSCMSK=11`** 同时启用 ATT_THR(bit1)；若未来想启用 Disk Theory(bit2)，需先关闭 bit1（源码注释明确两者不可共存）
5. **`Q_TAILSIT_DSKLD=18` 和 `Q_TAILSIT_MIN_VO=4`** 当前因 `GSCMSK` 未启用 bit2 而**实际不生效**，可保留作为未来切换 Disk Theory 时的参考
