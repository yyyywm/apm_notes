# STICK_MIXING 参数详解

## 概述

`STICK_MIXING` 是 ArduPlane 的一个全局参数，用于控制**在自动飞行模式下，飞行员是否可以用遥控器摇杆临时干预飞机控制**。

- 参数路径：`plane.g.stick_mixing`
- 默认值：`1` (FBW)
- 通过地面站（Mission Planner / QGroundControl）设置

---

## 参数取值

| 值 | 名称 | 说明 |
|----|------|------|
| `0` | `NONE` | 无混控，完全自动控制 |
| `1` | `FBW` | Fly-By-Wire 混控，摇杆控制滚转+俯仰姿态角 |
| `2` | `DIRECT_REMOVED` | 已废弃，原 DIRECT 混控功能已移除 |
| `3` | `VTOL_YAW` | 仅 VTOL 模式下偏航摇杆有效 |
| `4` | `FBW_NO_PITCH` | 仅滚转可混控，俯仰完全自动 |

---

## 核心机制

### 1. 全局参数，分散处理

虽然参数是全局的，但**各飞行模式根据自身逻辑决定是否使用、如何使用**。

### 2. 关键判断函数

```cpp
// Attitude.cpp:82
bool Plane::stick_mixing_enabled(void)
```

判断逻辑：
- 无遥控信号 → 禁用
- 自动模式（Auto/RTL/Loiter/Guided）→ 检查 `STICK_MIXING` 参数
- 非自动模式 → 总是启用（但模式本身可能直接控制）

### 3. 混控实现

```cpp
// mode.cpp:259
void Mode::run() {
    switch (plane.g.stick_mixing) {
        case FBW:
        case FBW_NO_PITCH:
        case DIRECT_REMOVED:
            plane.stabilize_stick_mixing_fbw();  // 执行混控
            break;
        case NONE:
        case VTOL_YAW:
            break;  // 不混控
    }
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}
```

---

## 各模式实际表现

### 手动/半自动模式（Stabilize / FBWA / FBWB / Manual / Acro / Training）

| 参数 | 效果 |
|------|------|
| 任何值 | **摇杆直接控制**，`STICK_MIXING` 不影响 |

这些模式本身就不是自动导航，没有"混控"概念。

### 固定翼自动模式（Auto / RTL / Cruise / Circle）

| 参数 | 效果 |
|------|------|
| `NONE (0)` | ❌ 无混控，完全自动 |
| `FBW (1)` | ✅ 滚转+俯仰都可干预 |
| `FBW_NO_PITCH (4)` | ✅ 仅滚转可干预，俯仰完全自动 |
| `VTOL_YAW (3)` | ❌ 无混控（对固定翼模式无效）|

### Loiter 模式（特殊）

- 若开启 `ENABLE_LOITER_ALT_CONTROL`：
  - 俯仰摇杆用于**控制高度**，不再参与姿态混控
  - 滚转摇杆仍按 FBW 方式混控

### Guided 模式

- 与 Auto 模式相同处理
- `NONE` 时完全自动，无混控

### VTOL 模式（QStabilize / QHover / QLoiter / QRTL / QLand）

| 参数 | 效果 |
|------|------|
| `NONE (0)` | ❌ 无混控；QRTL/Guided/VTOL Auto 下偏航也禁用 |
| `FBW (1)` | ✅ VTOL 控制器主导 + 滚转/俯仰混控 |
| `FBW_NO_PITCH (4)` | ✅ 仅滚转混控 |
| `VTOL_YAW (3)` | ✅ 偏航摇杆有效（滚转/俯仰由 VTOL 控制器处理）|

**QRTL 特殊**：仅在进近阶段（AIRBRAKE / APPROACH）允许混控。

### 过渡期间（Transition）

- 尾座式（Tailsitter）过渡中：强制禁用混控
- 其他 VTOL 过渡：由 `transition->allow_stick_mixing()` 决定

---

## 混控方式（FBW 模式）

```cpp
// Attitude.cpp:299
void Plane::stabilize_stick_mixing_fbw() {
    // 滚转：摇杆输入加到目标滚转角
    nav_roll_cd += roll_input * roll_limit_cd;

    // 俯仰：摇杆输入加到目标俯仰角（FBW_NO_PITCH 时跳过）
    if (plane.g.stick_mixing == StickMixing::FBW_NO_PITCH) return;
    nav_pitch_cd += pitch_input * pitch_range;
}
```

- 摇杆解释为**目标姿态角偏移**
- 松杆后飞机回到自动驾驶计算的原始航线
- 不改变航线目标，仅临时修正姿态

---

## 安全保护

以下情况强制禁用混控：

1. **无遥控信号**（`!rc().has_valid_input()`）
2. **QRTL 降落阶段**（`poscontrol.get_state() >= QPOS_POSITION1`）
3. **VTOL 降落位置控制**（`in_vtol_land_poscontrol()`）
4. **尾座式过渡中**（`in_vtol_transition()`）
5. **RC Failsafe + FBWA 滑翔模式**

---

## 相关代码文件

| 文件 | 作用 |
|------|------|
| `ArduPlane/defines.h:53` | 枚举定义 `StickMixing` |
| `ArduPlane/Parameters.cpp:96` | 参数注册，默认值 `FBW` |
| `ArduPlane/Parameters.h:450` | 参数声明 `AP_Enum<StickMixing> stick_mixing` |
| `ArduPlane/Attitude.cpp:82` | `stick_mixing_enabled()` 判断函数 |
| `ArduPlane/Attitude.cpp:299` | `stabilize_stick_mixing_fbw()` 混控实现 |
| `ArduPlane/mode.cpp:259` | `Mode::run()` 统一调用混控 |
| `ArduPlane/mode_loiter.cpp` | Loiter 特殊高度控制逻辑 |
| `ArduPlane/mode_qrtl.cpp` | QRTL 进近阶段混控 |
| `ArduPlane/quadplane.cpp:1389` | VTOL 偏航控制逻辑 |
| `ArduPlane/tailsitter.cpp:975` | 尾座式过渡混控限制 |

---

## 一句话总结

> `STICK_MIXING` 是全局参数，默认 `FBW`。自动模式下摇杆可临时修正姿态（不改变航线），各模式根据自身逻辑决定是否启用及如何混控，特殊阶段（降落、过渡）强制禁用。