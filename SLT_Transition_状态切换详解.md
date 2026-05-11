# SLT_Transition 状态切换详解

## 状态定义

```cpp
// ArduPlane/transition.h:111-115
enum class State {
    AIRSPEED_WAIT = 0,   // 等待空速达标
    TIMER         = 1,   // 空速达标后，按时间衰减多旋翼推力
    DONE          = 2,   // 转换完成，完全固定翼
};
```

---

## 一、AIRSPEED_WAIT → TIMER 的切换条件

### 1.1 正常路径（空速达标）

```cpp
// ArduPlane/quadplane.cpp:1636
if (have_airspeed && aspeed > plane.aparm.airspeed_min && !quadplane.assisted_flight) {
    transition_state = State::TIMER;
    airspeed_reached_tilt = quadplane.tiltrotor.current_tilt;
    gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", (double)aspeed);
}
```

**条件**：
- `have_airspeed` — 有有效的空速数据
- `aspeed > plane.aparm.airspeed_min` — 空速超过最小飞行空速
- `!quadplane.assisted_flight` — 辅助飞行未激活（防止重复进入）

**关键机制**：`assisted_flight` 在 `should_assist()` 返回 false 时（空速已够高）变为 false，此时满足 `!assisted_flight` 条件，触发进入 TIMER。

### 1.2 失败保护路径（倾斜旋翼超时强制转换）

```cpp
// ArduPlane/quadplane.cpp:1602-1615
// 转换超时检测
if (transition_start_ms != 0 &&
    (quadplane.transition_failure.timeout > 0) &&
    ((now - transition_start_ms) > ((uint32_t)quadplane.transition_failure.timeout * 1000))) {

    // 倾斜旋翼 + 地速够 + TRANS_FAIL_TO_FW 选项
    const bool tiltrotor_with_ground_speed = quadplane.tiltrotor.enabled()
        && (plane.ahrs.groundspeed() > plane.aparm.airspeed_min * 0.5);

    if (quadplane.option_is_set(QuadPlane::Option::TRANS_FAIL_TO_FW)
        && tiltrotor_with_ground_speed) {
        transition_state = State::TIMER;
        in_forced_transition = true;   // 标记为强制转换
    }
}
```

**条件**：
- 转换时间超过 `Q_TRANS_FAIL` 设置的超时时间
- 设置了 `TRANS_FAIL_TO_FW` 选项
- **倾斜旋翼**机型且地速 > 0.5 * 最小空速

**为什么倾斜旋翼特殊**：倾斜旋翼的电机可以向前倾斜提供推力，即使空速不足，靠地速也能维持飞行，所以可以强制完成转换。4+1传统VTOL不满足此条件，会走 QLAND/QRTL 紧急处理。

---

## 二、TIMER → DONE 的切换条件

```cpp
// ArduPlane/quadplane.cpp:1682-1698
const uint32_t transition_timer_ms = now - transition_low_airspeed_ms;
const float trans_time_ms = constrain_float(quadplane.transition_time_ms, 500, 30000);
const bool tilt_fwd_complete = !quadplane.tiltrotor.enabled()
    || quadplane.tiltrotor.tilt_angle_achieved();

if (transition_timer_ms > unsigned(trans_time_ms) && tilt_fwd_complete) {
    transition_state = State::DONE;
    in_forced_transition = false;
    transition_start_ms = 0;
    transition_low_airspeed_ms = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "Transition done");
}
```

**条件**：
1. `transition_timer_ms > Q_TRANSITION_MS`（默认5000ms）— TIMER阶段时间到了
2. `tilt_angle_achieved()` — 倾斜旋翼已完全前倾（非倾斜旋翼永远为true）

**TIMER阶段的行为**：
- 多旋翼推力按时间比例衰减：`throttle_scaled = last_throttle * transition_scale`
- 固定翼控制器逐渐接管
- 等待 `Q_TRANSITION_MS` 时间后完全切换到固定翼

---

## 三、直接到 DONE 的路径（跳过TIMER）

```cpp
// ArduPlane/quadplane.cpp:1569-1583
if (quadplane.tiltrotor.fully_fwd() && transition_state != State::AIRSPEED_WAIT) {
    if (transition_state == State::TIMER) {
        // 处理倾斜旋翼油门平滑过渡
        float throttle;
        if (plane.quadplane.tiltrotor.get_forward_throttle(throttle)) {
            plane.TECS_controller.set_throttle_min(throttle, true);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100);
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
    }
    transition_state = State::DONE;
    transition_start_ms = 0;
    transition_low_airspeed_ms = 0;
}
```

**条件**：
- `tiltrotor.fully_fwd()` — 倾斜旋翼已经完全前倾
- `transition_state != AIRSPEED_WAIT` — 不在等待空速阶段

**场景**：倾斜旋翼机型在TIMER阶段，如果旋翼已经完全前倾到固定翼位置，可以直接认为转换完成。

---

## 四、状态切换流程图

```
开始转换（从VTOL模式切换到固定翼模式）
    │
    ▼
┌─────────────────────────────────────────┐
│         AIRSPEED_WAIT                   │
│  • 多旋翼电机全推力工作                   │
│  • 推进电机逐渐加速                       │
│  • 等待空速达标                          │
│  • 同时检测转换超时（Q_TRANS_FAIL）       │
└──────────────┬──────────────────────────┘
               │
    ┌──────────┼──────────┐
    │          │          │
    ▼          ▼          ▼
 空速达标    超时强制     异常
(正常路径)  (倾斜旋翼)   (4+1机型)
    │          │          │
    ▼          ▼          ▼
┌─────────┐ ┌─────────┐  ┌─────────┐
│  TIMER  │ │  TIMER  │  │ QLAND/  │
│(正常)   │ │(强制)   │  │ QRTL    │
│         │ │in_forced│  │(紧急降落)│
│         │ │= true   │  │         │
└────┬────┘ └────┬────┘  └─────────┘
     │           │
     └───────────┘
                 │
    ┌────────────┼────────────┐
    │            │            │
    ▼            ▼            ▼
 时间到+      倾斜完成      直接完成
倾斜完成    (倾斜旋翼)    (fully_fwd)
    │            │            │
    └────────────┴────────────┘
                 │
                 ▼
        ┌─────────────┐
        │    DONE     │
        │  转换完成    │
        │ 纯固定翼模式 │
        └─────────────┘
```

---

## 五、关键变量说明

| 变量 | 定义位置 | 用途 |
|------|---------|------|
| `transition_start_ms` | transition.h:118 | 进入AIRSPEED_WAIT的时间，用于超时检测 |
| `transition_low_airspeed_ms` | transition.h:119 | 进入TIMER的时间，用于TIMER倒计时 |
| `airspeed_reached_tilt` | transition.h:129 | 进入TIMER时的倾斜角度，用于倾斜旋翼油门过渡 |
| `in_forced_transition` | transition.h:131 | 标记是否为强制转换（超时触发） |
| `Q_TRANSITION_MS` | quadplane.cpp:58 | TIMER阶段持续时间（默认5000ms） |
| `Q_TRANS_FAIL` | quadplane.h:347 | 转换失败超时时间（秒） |

---

## 六、各状态的行为差异

### AIRSPEED_WAIT
- 多旋翼电机：`THROTTLE_UNLIMITED`（全推力）
- 固定翼控制器：TECS使用合成空速
- 姿态控制：多旋翼控制器主导，`hold_hover()` 保持悬停
- 俯仰限制：`Q_TRAN_PIT_MAX` 限制最大俯仰角

### TIMER
- 多旋翼电机：推力按时间比例衰减
- 固定翼控制器：逐步接管，油门积分器重置
- 姿态控制：混合控制，多旋翼辅助稳定
- 目标：在 `Q_TRANSITION_MS` 时间内平滑过渡

### DONE
- 多旋翼电机：`SHUT_DOWN`（关闭）
- 固定翼控制器：完全接管
- 姿态控制：纯固定翼控制
- 转换完成，可以正常固定翼飞行

---

## 七、代码文件索引

| 文件 | 关键代码位置 | 说明 |
|------|-------------|------|
| `ArduPlane/transition.h` | 111-115 | State枚举定义 |
| `ArduPlane/quadplane.cpp` | 1532-1744 | `SLT_Transition::update()` 完整实现 |
| `ArduPlane/quadplane.cpp` | 1594 | `State::AIRSPEED_WAIT` 处理 |
| `ArduPlane/quadplane.cpp` | 1636 | AIRSPEED_WAIT → TIMER 切换（正常） |
| `ArduPlane/quadplane.cpp` | 1613 | AIRSPEED_WAIT → TIMER 切换（强制） |
| `ArduPlane/quadplane.cpp` | 1678 | `State::TIMER` 处理 |
| `ArduPlane/quadplane.cpp` | 1685 | TIMER → DONE 切换 |
| `ArduPlane/quadplane.cpp` | 1733 | `State::DONE` 处理 |
| `ArduPlane/quadplane.cpp` | 1569 | 直接到DONE的路径 |

---

> 生成时间: 2026-05-11
> 基于 ArduPlane SLT_Transition 代码分析
