# 垂起机型（QuadPlane）降落阶段逻辑详解

> 基于 ArduPlane 代码分析，讲解**普通垂起（四旋翼+固定翼）**和**尾座式垂起（Tailsitter）**的降落阶段状态流转、减速逻辑、旋翼保护机制。
>
> 核心代码文件：`ArduPlane/quadplane.cpp`、`ArduPlane/quadplane.h`、`ArduPlane/tailsitter.cpp`、`ArduPlane/tailsitter.h`

---

## 一、降落状态机（position_control_state）

定义在 [quadplane.h:521-531](../ArduPlane/quadplane.h#L521-L531)

```cpp
enum position_control_state {
    QPOS_NONE = 0,
    QPOS_APPROACH,       // 固定翼方式接近降落点
    QPOS_AIRBRAKE,       // 用VTOL旋翼当空气刹车减速
    QPOS_POSITION1,      // VTOL位置控制减速阶段
    QPOS_POSITION2,      // 水平定位到降落点正上方
    QPOS_LAND_DESCEND,   // 开始垂直下降
    QPOS_LAND_ABORT,     // 中止爬升
    QPOS_LAND_FINAL,     // 最终慢速下降到触地
    QPOS_LAND_COMPLETE   // 降落完成，锁定电机
};
```

**完整降落流程：**

```
固定翼飞行
    ↓
QPOS_APPROACH（固定翼接近）
    ↓ 距离 < 停止距离
QPOS_AIRBRAKE（旋翼空气刹车）
    ↓ 空速/姿态满足条件
QPOS_POSITION1（VTOL减速）
    ↓ 距离 < 10m 且 速度足够低
QPOS_POSITION2（水平定位）
    ↓ 位置误差 < 2m 且 水平速度 < 3m/s
QPOS_LAND_DESCEND（垂直下降）
    ↓ 高度 < Q_LAND_FINAL_ALT（默认6m）
QPOS_LAND_FINAL（0.5m/s慢速下降）
    ↓ land_detector检测到触地
QPOS_LAND_COMPLETE（锁定电机）
```

---

## 二、各阶段详细逻辑

### 2.1 QPOS_APPROACH — 固定翼接近阶段

**入口函数：** [quadplane.cpp:2269-2302](../ArduPlane/quadplane.cpp#L2269-L2302) `poscontrol_init_approach()`

飞机以**固定翼模式**飞行，使用TECS控制油门和俯仰，导航到降落点。

**核心逻辑：**

```cpp
// 计算停止距离 = v²/(2*a) + 2秒余量
const float stop_distance = stopping_distance_m() + 2*closing_speed_ms;

// 当距离降落点小于停止距离时，进入下一阶段
if (poscontrol.get_state() == QPOS_APPROACH && distance_m < stop_distance) {
    // 普通垂起机型（非尾座机）进入 AIRBRAKE
    poscontrol.set_state(QPOS_AIRBRAKE);
}
```

**停止距离计算：** [quadplane.cpp:4139-4145](../ArduPlane/quadplane.cpp#L4139-L4145)

```cpp
float QuadPlane::stopping_distance_m(float ground_speed_squared_m) const
{
    // v²/(2*a)，基于 Q_TRANS_DECEL 参数（默认2.0 m/s²）
    return ground_speed_squared_m / (2 * transition_decel_mss);
}
```

> **物理意义：** 假设匀减速运动，从当前地速减速到0需要的距离。
>
> 参数 `Q_TRANS_DECEL` 默认 **2.0 m/s²**，用户可配置。

---

### 2.2 QPOS_AIRBRAKE — 空气刹车阶段

**仅普通垂起机型使用**（尾座机跳过此阶段），[quadplane.cpp:2486-2490](../ArduPlane/quadplane.cpp#L2486-L2490)

```cpp
if (!suppress_z_controller && poscontrol.get_state() == QPOS_AIRBRAKE) {
    hold_hover(0);  // 悬停油门，用旋翼产生反向推力减速
    suppress_z_controller = true;  // 抑制Z轴控制器
}
```

**作用：** 在飞机仍保持固定翼姿态时，**提前启动VTOL旋翼**，利用旋翼的反向推力来快速减速。这样切换到VTOL模式时更平滑。

**退出条件：** [quadplane.cpp:2529-2536](../ArduPlane/quadplane.cpp#L2529-L2536)

空气刹车至少持续 **1000ms** 后，满足以下任一条件即进入 POSITION1：

| 条件 | 说明 |
|------|------|
| 空速 < 阈值 | 低于辅助速度 |
| 地速方向偏差 > 60° | 飞偏了 |
| 地速过快 | > 期望速度的1.2倍 |
| 地速过慢 | < 期望速度的0.5倍 |
| 姿态误差过大 | 横滚或俯仰误差 > 10° |

---

### 2.3 QPOS_POSITION1 — VTOL减速阶段

**核心减速逻辑：** [quadplane.cpp:2601-2721](../ArduPlane/quadplane.cpp#L2601-L2721)

这是**最关键的阶段**，飞机从固定翼模式过渡到VTOL模式，同时进行减速。

#### 2.3.1 目标速度计算

```cpp
// 计算到达position2时应有的速度
// v = √(2*a*s + v₀²)
const float stopping_speed_ms = safe_sqrt(
    MAX(0, wp_distance_m - position2_dist_threshold_m) * 2 * transition_decel_mss
    + sq(position2_target_speed_ms)
);
```

> **物理意义：** 根据当前距离降落点的距离，计算应该保持多大的地速，才能在到达POSITION2阈值时刚好减速到目标速度（3 m/s）。

#### 2.3.2 目标加速度计算

```cpp
// 计算需要的减速度
const float approach_accel_mss = MIN(
    accel_needed(wp_distance_m, sq(closing_groundspeed_ms)),
    transition_decel_mss * 2  // 不超过2倍Q_TRANS_DECEL
);
```

#### 2.3.3 速度-加速度控制

```cpp
// 设置目标速度和加速度
Vector2f diff_wp_norm = wp_distance_ne_m.normalized();
target_speed_ne_ms = diff_wp_norm * approach_speed_ms;
target_accel_ne_mss = diff_wp_norm * (-approach_accel_mss);

// 使用输入整形，遵守加加速度限制
pos_control->input_vel_accel_NE_m(target_speed_ne_ms, target_accel_ne_mss);

// 只控制速度和加速度，不控制位置
pos_control->NE_stop_pos_stabilisation();

// 运行水平速度控制器
run_xy_controller(MAX(approach_accel_mss, transition_decel_mss)*1.5);
```

> **关键点：**
> - 使用 `transition_decel_mss`（参数 `Q_TRANS_DECEL`，默认 2.0 m/s²）作为减速基准
> - `NE_stop_pos_stabilisation()` 关闭位置稳定，**只跟踪速度曲线**
> - 这保证了减速过程平滑，不会因为位置误差导致飞机加速追赶

#### 2.3.4 前拉电机辅助减速

[quadplane.cpp:3075-3087](../ArduPlane/quadplane.cpp#L3075-L3087)

```cpp
float fwd_thr_scaler;
if (!in_vtol_land_approach()) {
    // 接近地面时关闭前拉电机，防止打桨
    float height_above_ground_m = plane.relative_ground_altitude(...);
    fwd_thr_scaler = linear_interpolate(0.0f, 1.0f, height_above_ground_m,
                                        alt_cutoff_m, alt_cutoff_m + 2);
} else {
    // 水平定位阶段始终允许前拉电机运行
    fwd_thr_scaler = 1.0f;
}
q_fwd_throttle *= fwd_thr_scaler;
```

> 在POSITION1阶段（属于`in_vtol_land_approach()`），前拉电机**始终允许运行**，帮助减速。
>
> 但在接近地面时（非approach阶段），前拉电机会被线性关闭防止打桨。

#### 2.3.5 进入POSITION2的条件

[quadplane.cpp:2758-2766](../ArduPlane/quadplane.cpp#L2758-L2766)

```cpp
if ((plane.auto_state.wp_distance < position2_dist_threshold_m) &&
    fabsf(rel_groundspeed_sq) < sq(3 * position2_target_speed_ms)) {
    poscontrol.set_state(QPOS_POSITION2);
}
```

- 距离降落点 **< 10米**
- 地速平方 **< (3 × 3)² = 81**（即地速 < 9 m/s）

---

### 2.4 QPOS_POSITION2 — 水平定位阶段

飞机在降落点**正上方悬停**，进行最后的水平位置调整。

**进入QPOS_LAND_DESCEND的条件：** [quadplane.cpp:3658-3697](../ArduPlane/quadplane.cpp#L3658-L3697)

```cpp
if (poscontrol.get_state() == QPOS_POSITION2) {
    // 水平位置误差 < 2米，且水平速度 < 3m/s
    if (reached_position && (vel_ned_ms.xy() - approach_vel_ne_ms).length() < 3.0) {
        poscontrol.set_state(QPOS_LAND_DESCEND);
        pos_control->set_lean_angle_max_cd(0);  // 限制倾斜角为0
        // 放下起落架
        plane.g2.landing_gear.deploy_for_landing();
    }
}
```

---

### 2.5 QPOS_LAND_DESCEND — 垂直下降阶段

**下降速率控制：** [quadplane.cpp:1314-1361](../ArduPlane/quadplane.cpp#L1314-L1361)

```cpp
float QuadPlane::landing_descent_rate_ms(float height_above_ground_m)
{
    // 在 land_final_alt_m（默认6米）到 land_final_alt_m+6米之间线性插值
    float ret_ms = linear_interpolate(land_final_speed_ms,        // 0.5 m/s
                                      wp_nav->get_default_speed_down_ms(),
                                      height_above_ground_m,
                                      land_final_alt_m,           // 6米
                                      land_final_alt_m + 6);      // 12米
    // ...
}
```

**下降速率曲线：**

| 高度 | 下降速率 |
|------|----------|
| > 12米 | 默认下降速度（如 1.5 m/s） |
| 6 ~ 12米 | 线性减速 |
| < 6米 | 0.5 m/s（Q_LAND_FINAL_SPD） |

---

### 2.6 QPOS_LAND_FINAL — 最终降落阶段

**进入条件：** [quadplane.cpp:3629-3647](../ArduPlane/quadplane.cpp#L3629-L3647)

```cpp
bool QuadPlane::check_land_final(void)
{
    float height_above_ground_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
    // 需要连续两次测距读数在5米内变化，且高度低于 Q_LAND_FINAL_ALT
    if (height_above_ground_m < land_final_alt_m &&
        fabsf(height_above_ground_m - last_land_final_agl_m) < 5) {
        return true;
    }
    // 或者着陆检测器检测到已着陆
    return land_detector(6000);
}
```

进入后：
- 以 **0.5 m/s**（`Q_LAND_FINAL_SPD`）慢速下降
- 关闭内燃机（如果有，`land_icengine_cut` 参数）

---

### 2.7 QPOS_LAND_COMPLETE — 降落完成

**着陆检测器：** [quadplane.cpp:3558-3591](../ArduPlane/quadplane.cpp#L3558-L3591)

```cpp
bool QuadPlane::land_detector(uint32_t timeout_ms)
{
    // 1. 电机在最低油门限制，且油门混合最小
    bool might_be_landed = should_relax() && !poscontrol.pilot_correction_active;
    if (!might_be_landed) return false;

    // 2. 垂直位置在 timeout_ms 时间内变化不超过 20cm
    if (fabsf(height_m - landing_detect.vpos_start_m) > 0.2) return false;

    // 3. 电机在最低限制持续 timeout_ms+1000
    if ((now - landing_detect.land_start_ms) < timeout_ms) return false;
    if ((now - landing_detect.lower_limit_start_ms) < (timeout_ms+1000)) return false;

    return true;
}
```

**`should_relax()` 判断：** [quadplane.cpp:1257-1276](../ArduPlane/quadplane.cpp#L1257-L1276)

```cpp
bool QuadPlane::should_relax(void)
{
    // 电机在最低油门限制，且油门混合最小
    bool motor_at_lower_limit = motors->limit.throttle_lower
                                && attitude_control->is_throttle_mix_min();
    if (motors->get_throttle() < 0.01f) {
        motor_at_lower_limit = true;
    }

    if (!motor_at_lower_limit) {
        landing_detect.lower_limit_start_ms = 0;
        return false;
    } else if (landing_detect.lower_limit_start_ms == 0) {
        landing_detect.lower_limit_start_ms = tnow;
    }

    // 需要持续1000ms
    return (tnow - landing_detect.lower_limit_start_ms) > 1000;
}
```

**着陆检测三要素：**

1. **电机在最低限制**（`motors->limit.throttle_lower`）且 **油门混合最小**（`is_throttle_mix_min()`）
2. **高度不变**（4000ms内变化 < 20cm）
3. **持续时间**（电机最低限制持续 4000+1000=5000ms）

确认降落后：
- 发送"Land complete"消息
- 自动锁定电机（除非 `MIS_OPTIONS` 设置了降落后继续）

---

## 三、旋翼保护机制

### 3.1 前拉电机近地保护

[quadplane.cpp:3075-3087](../ArduPlane/quadplane.cpp#L3075-L3087)

```cpp
if (!in_vtol_land_approach()) {
    // 防止前拉螺旋桨打地
    float alt_cutoff_m = MAX(0, vel_forward_alt_cutoff_m);
    float height_above_ground_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
    // 在 cutoff 高度到 cutoff+2米之间线性降低前拉油门到0
    fwd_thr_scaler = linear_interpolate(0.0f, 1.0f,
                                        height_above_ground_m,
                                        alt_cutoff_m, alt_cutoff_m + 2);
} else {
    // 水平定位阶段始终允许前拉电机
    fwd_thr_scaler = 1.0f;
}
```

**逻辑：**
- 在 `QPOS_APPROACH` / `QPOS_AIRBRAKE` / `QPOS_POSITION1` / `QPOS_POSITION2` 阶段（`in_vtol_land_approach()`返回true），前拉电机**始终允许运行**
- 进入 `QPOS_LAND_DESCEND` 后，接近地面时前拉电机**线性关闭**

### 3.2 油门混合保护（降落时姿态优先）

[quadplane.cpp:4177-4235](../ArduPlane/quadplane.cpp#L4177-L4235)

```cpp
void QuadPlane::update_throttle_mix(void)
{
    // ...
    if (in_vtol_land_sequence()) {
        // 在降落序列中，除了最终阶段外都使用最大油门混合
        use_mix_max = !in_vtol_land_final();
    }
    // ...
}
```

**逻辑：**
- 在 `QPOS_APPROACH` ~ `QPOS_LAND_DESCEND` 阶段：`use_mix_max = true`
- 在 `QPOS_LAND_FINAL` 阶段：`use_mix_max = false`

> **含义：** 在降落过程中（除最终阶段外），旋翼会**优先保证姿态控制**，而不是省油。这保证了降落过程中即使接近地面，旋翼也有足够的控制力矩保持姿态稳定。
>
> 进入 `QPOS_LAND_FINAL` 后，才允许油门混合最小化，让飞机"软着陆"。

---

## 四、关键参数汇总

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Q_TRANS_DECEL` | 2.0 m/s² | 过渡减速率，控制POSITION1减速曲线 |
| `Q_LAND_FINAL_ALT` | 6.0 m | 进入最终降落的AGL高度 |
| `Q_LAND_FINAL_SPD` | 0.5 m/s | 最终降落下降速度 |
| `Q_LAND_ALTCHG` | 0.2 m | 着陆检测高度变化阈值 |
| `Q_LAND_SPEED` | - | 接近阶段目标速度 |
| `Q_WP_SPEED` | - | VTOL默认水平飞行速度 |
| `Q_OPTIONS` bit 21 | - | 未解锁时倾转电机向上防打桨（倾转机） |
| `Q_OPTIONS` bit 10 | - | 未解锁时允许偏航倾转 |

---

## 五、流程图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         固定翼飞行                                   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  QPOS_APPROACH                                                      │
│  - 固定翼方式接近降落点                                              │
│  - TECS控制油门/俯仰                                                 │
│  - 当距离 < 停止距离(v²/2a + 2s) 时进入AIRBRAKE                     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  QPOS_AIRBRAKE                                                      │
│  - VTOL旋翼启动，产生反向推力减速                                     │
│  - 飞机仍保持固定翼姿态                                              │
│  - 持续至少1000ms后，满足条件进入POSITION1                           │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  QPOS_POSITION1                                                     │
│  - VTOL位置控制器接管                                                │
│  - 按 v = √(2as) 曲线减速                                            │
│  - 使用 Q_TRANS_DECEL (默认2.0) 作为减速度基准                        │
│  - 前拉电机可辅助减速                                                │
│  - 油门混合最大（姿态优先）                                          │
│  - 距离 < 10m 且 速度 < 9m/s 进入POSITION2                           │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  QPOS_POSITION2                                                     │
│  - 水平定位到降落点正上方                                            │
│  - 位置误差 < 2m 且 水平速度 < 3m/s 进入LAND_DESCEND                 │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  QPOS_LAND_DESCEND                                                  │
│  - 垂直下降，起落架放下                                              │
│  - 下降速率：>12m时1.5m/s → 6m时线性减速 → <6m时0.5m/s              │
│  - 高度 < Q_LAND_FINAL_ALT(6m) 进入LAND_FINAL                        │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  QPOS_LAND_FINAL                                                    │
│  - 0.5m/s 慢速最终下降                                               │
│  - 油门混合最小化（允许软着陆）                                       │
│  - land_detector(4000) 检测触地                                      │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│  QPOS_LAND_COMPLETE                                                 │
│  - 着陆检测器确认：                                                  │
│    1. 电机在最低限制 + 油门混合最小                                   │
│    2. 高度4000ms内变化 < 20cm                                        │
│    3. 电机最低限制持续 5000ms                                        │
│  - 发送"Land complete"，锁定电机                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 六、核心代码定位速查

| 功能 | 文件 | 行号 |
|------|------|------|
| 状态机定义 | `quadplane.h` | 521-531 |
| 接近阶段初始化 | `quadplane.cpp` | 2269-2302 |
| 停止距离计算 | `quadplane.cpp` | 4139-4171 |
| 空气刹车阶段 | `quadplane.cpp` | 2434-2516 |
| POSITION1减速 | `quadplane.cpp` | 2601-2768 |
| 期望接近速度 | `quadplane.cpp` | 4398-4428 |
| 下降速率控制 | `quadplane.cpp` | 1314-1361 |
| 着陆检测器 | `quadplane.cpp` | 3558-3591 |
| 着陆完成检测 | `quadplane.cpp` | 3596-3623 |
| 最终降落判断 | `quadplane.cpp` | 3629-3647 |
| 降落验证主函数 | `quadplane.cpp` | 3652-3734 |
| 前拉电机保护 | `quadplane.cpp` | 3075-3087 |
| 油门混合控制 | `quadplane.cpp` | 4177-4235 |
| 粗糙着陆检测 | `quadplane.cpp` | 1257-1276 |

---

## 七、尾座式垂起（Tailsitter）降落特殊逻辑

尾座式垂起和普通垂起共用同一套状态机，但在降落流程中有几个关键区别。

### 7.1 尾座式简介

尾座式飞机的特点是：
- **固定翼飞行时**：机身水平，像普通固定翼一样飞行
- **VTOL飞行时**：机身竖直（机头朝上），靠旋翼产生升力
- **没有独立的前拉电机**，旋翼同时承担固定翼推进和VTOL升力的功能
- **降落时需要从水平姿态翻转到竖直姿态**

### 7.2 最大的区别：没有 AIRBRAKE 阶段

尾座式**跳过 AIRBRAKE**，从 APPROACH **直接**到 POSITION1。

原因：尾座机没有独立的前拉电机，它的旋翼就是固定翼飞行的动力源，不存在"提前启动旋翼"的概念。

[quadplane.cpp:2498-2507](../ArduPlane/quadplane.cpp#L2498-L2507)

```cpp
if (poscontrol.get_state() == QPOS_APPROACH && distance_m < stop_distance) {
    if (tailsitter.enabled() || motors->get_desired_spool_state() == ...) {
        // tailsitters don't use airbrake stage for landing
        gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 v=%.1f ...");
        poscontrol.set_state(QPOS_POSITION1);
        transition->set_last_fw_pitch();
    } else {
        // 普通垂起进入 AIRBRAKE
        poscontrol.set_state(QPOS_AIRBRAKE);
    }
}
```

### 7.3 POSITION1 — 姿态过渡是关键

[quadplane.cpp:2604-2606](../ArduPlane/quadplane.cpp#L2604-L2606)

```cpp
case QPOS_POSITION1: {
    if (tailsitter.enabled() && tailsitter.in_vtol_transition(now_ms)) {
        break;  // 过渡期间暂停位置控制
    }
```

**尾座机在 POSITION1 期间同时进行两件事：**
1. **减速**：和普通垂起一样，按 `v = sqrt(2as)` 曲线减速
2. **姿态过渡**：飞机从水平姿态（固定翼）翻转到竖直姿态（VTOL）

`tailsitter.in_vtol_transition()` 返回 true 时，说明飞机还在翻转过程中，此时**暂停位置控制**，优先完成姿态过渡。

### 7.4 过渡完成判断

[tailsitter.cpp:553-586](../ArduPlane/tailsitter.cpp#L553-L586)

```cpp
bool Tailsitter::transition_vtol_complete(void) const
{
    // 零油门+地速<1m/s 时立即完成过渡（地面保护）
    if ((quadplane.get_pilot_throttle() < 0.05f) && _is_vectored) {
        if (quadplane.ahrs.groundspeed() < 1.0f) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done, zero throttle");
            return true;
        }
    }

    // 俯仰角超过过渡角度（默认45度）
    const float trans_angle = get_transition_angle_vtol();
    if (labs(plane.ahrs.pitch_sensor) > trans_angle*100) {
        return true;
    }

    // 横滚误差过大
    if (roll_cd > MAX(4500, plane.roll_limit_cd + 500)) {
        return true;
    }

    // 超时
    if (超时) return true;
}
```

**过渡完成条件：**
- 俯仰角 > `Q_TAILSIT_ANG_VT`（默认用 `Q_TAILSIT_ANGLE`，45度）
- 或横滚误差 > 限制
- 或超时
- 特殊：零油门+地速<1m/s 时立即完成（防止地面打桨）

### 7.5 Z轴控制器特殊处理

[quadplane.cpp:1091-1106](../ArduPlane/quadplane.cpp#L1091-L1106)

```cpp
void QuadPlane::run_z_controller(void)
{
    if (tailsitter.in_vtol_transition(now)) {
        // never run Z controller in tailsitter transition
        return;  // 过渡期间不运行Z控制器
    }

    if (!tailsitter.enabled()) {
        pos_control->D_init_controller();
    } else {
        // 尾座机初始化Z控制器时禁止下降
        pos_control->D_init_controller_no_descent();
    }
}
```

**尾座机特殊处理：**
- 过渡期间**不运行Z轴控制器**
- 初始化时**禁止下降**（`D_init_controller_no_descent`），防止过渡过程中掉高度

### 7.6 电机输出特殊处理

[quadplane.cpp:2041-2048](../ArduPlane/quadplane.cpp#L2041-L2048)

```cpp
if (tailsitter.in_vtol_transition(now) && !assisted_flight) {
    /*
      don't run the motor outputs while in tailsitter->vtol
      transition. That is taken care of by the fixed wing
      stabilisation code
    */
    return;
}
```

过渡期间**不运行多旋翼电机输出**，由固定翼稳定代码控制。

### 7.7 姿态控制切换

[quadplane.cpp:962](../ArduPlane/quadplane.cpp#L962)

```cpp
bool use_multicopter_control = in_vtol_mode() && !tailsitter.in_vtol_transition() && !force_fw_control_recovery;
```

尾座机只有在**不在过渡中**时才使用多旋翼控制。

### 7.8 尾座式 vs 普通垂起 对比表

| 特性 | 普通垂起（四旋翼+固定翼） | 尾座式（Tailsitter） |
|------|------------------------|---------------------|
| AIRBRAKE阶段 | 有 | **跳过** |
| 接近到减速过渡 | APPROACH -> AIRBRAKE -> POSITION1 | APPROACH -> **直接** POSITION1 |
| 减速方式 | 旋翼反向推力 + 前拉电机 | 主要靠气动减速 + 旋翼翻转 |
| 姿态过渡 | 无需（始终保持水平） | **需要翻转90度**（水平->竖直） |
| POSITION1期间 | 正常位置控制 | 过渡期间**暂停位置控制** |
| Z控制器 | 正常运行 | 过渡期间**关闭**，初始化禁止下降 |
| 电机输出 | 始终运行 | 过渡期间由固定翼代码接管 |
| 油门混合 | 降落时最大 | 相同 |
| 着陆检测 | 相同 | 相同 |

### 7.9 尾座式降落流程图

```
固定翼飞行（水平姿态）
    |
    v
QPOS_APPROACH（固定翼接近）
    | 距离 < 停止距离
    v
QPOS_POSITION1（减速 + 姿态过渡）
    - 按 v = sqrt(2as) 减速
    - 同时翻转：水平 -> 竖直（俯仰到90度）
    - 过渡期间：暂停位置控制、关闭Z控制器
    - 过渡完成后：恢复多旋翼控制
    | 距离 < 10m 且 速度足够低
    v
QPOS_POSITION2（竖直姿态悬停定位）
    | 位置误差 < 2m 且 水平速度 < 3m/s
    v
QPOS_LAND_DESCEND -> LAND_FINAL -> LAND_COMPLETE
    - 竖直姿态下降，和普通垂起相同逻辑
```

### 7.10 核心代码定位（尾座式）

| 功能 | 文件 | 行号 |
|------|------|------|
| 尾座机跳过AIRBRAKE | `quadplane.cpp` | 2499 |
| 尾座机POSITION1过渡暂停 | `quadplane.cpp` | 2604 |
| 尾座机Z控制器过渡关闭 | `quadplane.cpp` | 1091 |
| 尾座机电机输出过渡接管 | `quadplane.cpp` | 2041 |
| 尾座机过渡完成判断 | `tailsitter.cpp` | 553 |
| 尾座机VTOL过渡角度 | `tailsitter.cpp` | 40 |
| 尾座机姿态控制切换 | `quadplane.cpp` | 962 |

---

## 八、尾座式垂起过渡状态机（Tailsitter_Transition::State）

尾座机除了降落位置控制状态机（QPOS_*）之外，还有一套独立的**过渡状态机**，用于管理固定翼和VTOL模式之间的姿态翻转。

### 8.1 状态定义

[tailsitter.h:194-198](../ArduPlane/tailsitter.h#L194-L198)

```cpp
enum class State {
    ANGLE_WAIT_FW   = 0,   // 等待抬头到固定翼角度（VTOL->固定翼过渡）
    ANGLE_WAIT_VTOL = 1,   // 等待低头到VTOL角度（固定翼->VTOL过渡）
    DONE            = 2,   // 过渡完成，当前模式稳定
} transition_state;
```

### 8.2 VTOL -> 固定翼过渡（ANGLE_WAIT_FW）

**触发条件：** 从VTOL模式切换到固定翼模式（如TAKEOFF后切换AUTO）

**流程：**

```
VTOL竖直飞行
    |
    v
ANGLE_WAIT_FW
    - 强制机头下压，俯仰角线性减小
    - 目标：俯仰角 < Q_TAILSIT_ANGLE (默认45度)
    - 横滚：强制为0
    - 油门：max(悬停油门, 当前油门)
    - 多旋翼姿态控制运行
    |
    v 俯仰角 < 45度
transition_fw_complete() 返回 true
    |
    v
DONE（固定翼稳定飞行）
    - 设置前向俯仰限制 (fw_limit_start_ms)
    - 防止机头过快上抬
```

**代码：** [tailsitter.cpp:847-873](../ArduPlane/tailsitter.cpp#L847-L873)

```cpp
case State::ANGLE_WAIT_FW: {
    if (tailsitter.transition_fw_complete()) {
        transition_state = State::DONE;
        fw_limit_start_ms = now;
        fw_limit_initial_pitch = constrain_float(quadplane.ahrs.pitch_sensor,-8500,8500);
        break;
    }
    // 强制机头下压
    plane.nav_pitch_cd = constrain_float(
        fw_transition_initial_pitch - (transition_rate_fw * dt) * 0.1f,
        -8500, 8500);
    plane.nav_roll_cd = 0;
    // 运行多旋翼姿态控制
    quadplane.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(...);
    quadplane.motors_output();
}
```

### 8.3 固定翼 -> VTOL过渡（ANGLE_WAIT_VTOL）

**触发条件：** 从固定翼模式切换到VTOL模式（如QLAND/QRTL）

**流程：**

```
固定翼水平飞行
    |
    v 切换到VTOL模式
ANGLE_WAIT_VTOL
    - 固定翼代码接管控制
    - 强制机头上抬，俯仰角线性增加
    - 目标：俯仰角 > Q_TAILSIT_ANG_VT (默认45度)
    - 横滚：强制为0
    - 油门：恒定（悬停油门或巡航油门较大值）
    - 多旋翼输出被跳过
    - 位置控制暂停
    |
    v 俯仰角 > 45度
transition_vtol_complete() 返回 true
    |
    v
打印 "Transition VTOL done"
vtol_limit_start_ms = now
    |
    v
DONE（VTOL稳定飞行）
    - 设置VTOL俯仰限制 (vtol_limit_start_ms)
    - 防止机头过快下压
```

**进入 ANGLE_WAIT_VTOL：** [tailsitter.cpp:885-922](../ArduPlane/tailsitter.cpp#L885-L922)

```cpp
void Tailsitter_Transition::VTOL_update()
{
    if ((now - last_vtol_mode_ms) > 1000) {
        // 刚进入VTOL模式，设置过渡状态
        transition_state = State::ANGLE_WAIT_VTOL;
    }
    last_vtol_mode_ms = now;

    if (transition_state == State::ANGLE_WAIT_VTOL) {
        if (!quadplane.tailsitter.transition_vtol_complete()) {
            return;  // 过渡未完成，继续等待
        }
        // 过渡完成，设置VTOL俯仰限制
        vtol_limit_start_ms = now;
        vtol_limit_initial_pitch = quadplane.ahrs_view->pitch_sensor;
    }
    restart();  // 重置为固定翼过渡做准备
}
```

**ANGLE_WAIT_VTOL 期间的姿态控制：** [tailsitter.cpp:940-952](../ArduPlane/tailsitter.cpp#L940-L952)

```cpp
void Tailsitter_Transition::set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd)
{
    if (tailsitter.in_vtol_transition(now)) {
        // 过渡期间：强制抬头 + 横滚水平
        uint32_t dt = now - vtol_transition_start_ms;
        nav_pitch_cd = constrain_float(
            vtol_transition_initial_pitch + (tailsitter.transition_rate_vtol * dt) * 0.1f,
            -8500, 8500);
        nav_roll_cd = 0;
    }
}
```

### 8.4 过渡完成判断

**VTOL->固定翼完成条件** [tailsitter.cpp:527-547](../ArduPlane/tailsitter.cpp#L527-L547)

```cpp
bool Tailsitter::transition_fw_complete(void)
{
    // 俯仰角 < 过渡角度（默认45度）
    if (labs(quadplane.ahrs_view->pitch_sensor) > transition_angle_fw*100) {
        return true;
    }
    // 横滚误差过大
    if (labs(quadplane.ahrs_view->roll_sensor) > MAX(4500, plane.roll_limit_cd + 500)) {
        return true;
    }
    // 超时
    if (超时) return true;
}
```

**固定翼->VTOL完成条件** [tailsitter.cpp:553-586](../ArduPlane/tailsitter.cpp#L553-L586)

```cpp
bool Tailsitter::transition_vtol_complete(void) const
{
    // 未解锁时立即完成
    if (!plane.arming.is_armed_and_safety_off()) return true;

    // 零油门+地速<1m/s（仅矢量尾座机，地面保护）
    if ((quadplane.get_pilot_throttle() < 0.05f) && _is_vectored) {
        if (quadplane.ahrs.groundspeed() < 1.0f) return true;
    }

    // 俯仰角 > 过渡角度（默认45度）
    const float trans_angle = get_transition_angle_vtol();
    if (labs(plane.ahrs.pitch_sensor) > trans_angle*100) {
        return true;
    }

    // 横滚误差过大
    if (roll_cd > MAX(4500, plane.roll_limit_cd + 500)) {
        return true;
    }

    // 超时
    if (超时) return true;
}
```

### 8.5 过渡期间实际控制流

```
主循环 quadplane.update()
    |
    ├── 固定翼导航更新 (nav_controller->update_waypoint)
    |       └── 计算 nav_roll_cd, nav_pitch_cd
    |
    ├── Tailsitter_Transition::set_FW_roll_pitch()
    |       └── 覆盖俯仰角：按 rate 线性抬头/低头
    |       └── 覆盖横滚角：强制为0
    |
    ├── 固定翼稳定代码 (stabilize/calc_nav_roll_pitch)
    |       └── 控制舵面
    |
    ├── motors_output()
    |       └── 检查 tailsitter.in_vtol_transition()
    |       └── 是 → 直接返回（跳过多旋翼输出）
    |
    └── Tailsitter::output()
            └── 恒定油门输出到左右电机
            └── 方向舵居中
            └── 固定翼电机掩码
```

### 8.6 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Q_TAILSIT_ANGLE` | 45度 | 固定翼过渡角度阈值 |
| `Q_TAILSIT_ANG_VT` | 0（用Q_TAILSIT_ANGLE） | VTOL过渡角度 |
| `transition_rate_fw` | 自动计算 | 固定翼过渡速率 (deg/s) |
| `transition_rate_vtol` | 自动计算 | VTOL过渡速率 (deg/s) |
| `Q_TAILSIT_THR_VT` | -1 | 过渡油门（-1=自动） |

### 8.7 调试日志对应的状态流转

以用户提供的日志为例：

```
固定翼飞行
    |
    v 进入QLAND
QPOS_APPROACH（固定翼接近）
    |
    v 距离 < 停止距离(208m)
QPOS_POSITION1（VTOL位置控制）
    ├── transition_state = ANGLE_WAIT_VTOL  ← 进入过渡状态
    ├── 固定翼代码接管控制
    ├── 强制机头上抬
    ├── 位置控制暂停
    ├── 多旋翼输出跳过
    │
    v 俯仰角 > 45度
"Transition VTOL done"  ← 过渡完成
    ├── transition_state 变为 DONE
    ├── vtol_limit_start_ms 设置
    ├── 恢复多旋翼控制
    ├── 恢复位置控制
    │
    v 但此时已经冲到 d=5.7m
"VTOL Overshoot"  ← 冲过头
```

### 8.8 核心代码定位（过渡状态机）

| 功能 | 文件 | 行号 |
|------|------|------|
| 过渡状态定义 | `tailsitter.h` | 194-198 |
| VTOL->固定翼过渡 | `tailsitter.cpp` | 847-873 |
| 固定翼->VTOL过渡 | `tailsitter.cpp` | 885-922 |
| 固定翼过渡完成判断 | `tailsitter.cpp` | 527-547 |
| VTOL过渡完成判断 | `tailsitter.cpp` | 553-586 |
| 过渡期间姿态覆盖 | `tailsitter.cpp` | 940-952 |
| 过渡期间电机输出 | `tailsitter.cpp` | 285-341 |
| 多旋翼输出跳过 | `quadplane.cpp` | 2041-2048 |
| VTOL更新入口 | `tailsitter.cpp` | 885 |
| 固定翼更新入口 | `tailsitter.cpp` | 830 |
