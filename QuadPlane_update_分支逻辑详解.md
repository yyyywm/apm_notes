# QuadPlane::update() 分支逻辑详解

> 源文件：`ArduPlane/quadplane.cpp` 第1818-1844行
> 讲解 QuadPlane 核心更新函数中的模式分支判断逻辑

---

## 核心问题：当前是什么飞行模式？

这段代码首先判断飞机当前处于什么"大类"模式：

```
┌─────────────────────────────────────────┐
│  !in_vtol_mode() && !in_vtol_airbrake() │
│         当前是固定翼模式？                 │
└─────────────────────────────────────────┘
              │
      ┌───────┴───────┐
      ▼               ▼
   【是】            【否】
   固定翼模式        VTOL模式
   (走上面分支)     (走下面分支)
```

---

## 分支一：固定翼模式 [1818-1833]

```cpp
if (!in_vtol_mode() && !in_vtol_airbrake()) {
```

进入这个分支说明：飞机当前在**固定翼模式**飞行（如 FBWA、Cruise、Auto 等）。

### 然后进一步细分：

```
固定翼模式
    │
    ├── 手动/特技/训练模式？ ──► 多旋翼电机完全关闭
    │   (mode_manual/acro/training)
    │
    └── 其他自动模式？ ──► 处理 VTOL 转换
        (FBWA/Cruise/Auto等)
```

### 情况 A：手动/特技/训练 [1820-1830]

```cpp
if (plane.control_mode == &plane.mode_manual ||
    plane.control_mode == &plane.mode_acro ||
    plane.control_mode == &plane.mode_training) {

    // 尾座式？不，普通机型
    if (!tailsitter.enabled()) {
        // 多旋翼电机：关闭
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();  // 执行关闭
    }

    // 强制标记转换已完成（不需要转换）
    transition->force_transition_complete();

    // 不在辅助飞行状态
    assisted_flight = false;
}
```

**为什么手动模式要关闭多旋翼电机？**

| 原因 | 说明 |
|-----|------|
| 飞行员完全控制 | 手动模式下飞行员直接控制固定翼舵面 |
| 避免干扰 | 多旋翼电机转动会产生阻力，影响固定翼飞行性能 |
| 省电 | 不需要多旋翼辅助 |

> 尾座式除外：因为尾座式的"多旋翼电机"和"固定翼推进"是同一套电机。

### 情况 B：其他自动模式 [1832]

```cpp
} else {
    transition->update();  // 处理转换状态机
}
```

**什么时候会走到这里？**

例如：
- 你从 **QHover** 切换到 **FBWA**（固定翼模式）
- 飞机需要从多旋翼飞行**转换**到固定翼飞行
- `transition->update()` 管理这个转换过程：
  1. 等待空速达到转换速度
  2. 逐步增加固定翼舵面控制
  3. 逐步减少多旋翼辅助
  4. 标记转换完成

---

## 分支二：VTOL 模式 [1834-1844]

```cpp
} else {
    // 当前是 VTOL 模式（QStabilize/QHover/QLoiter等）
```

### 代码解析：

```cpp
// 检查是否在"空气刹车"阶段（VTOL降落前的减速阶段）
assisted_flight = in_vtol_airbrake();
// 如果是空气刹车，assisted_flight = true，固定翼舵面会辅助减速

// 输出多旋翼电机控制（这是 VTOL 飞行的核心）
motors_output();

// 设置转换状态，为下次进入固定翼模式做准备
transition->VTOL_update();
```

**`transition->VTOL_update()` 做什么？**

```cpp
// SLT_Transition::VTOL_update() [quadplane.cpp:1746]
void SLT_Transition::VTOL_update() {
    transition_start_ms = 0;           // 清除转换开始时间
    transition_low_airspeed_ms = 0;    // 清除低空速计时

    if (quadplane.throttle_wait && !plane.is_flying()) {
        // 如果在地面上且等待油门，标记转换已完成
        in_forced_transition = false;
        transition_state = State::DONE;
    } else {
        // 准备下次转换：设置为空速等待状态
        transition_state = State::AIRSPEED_WAIT;
    }
}
```

**作用**：为下次从 VTOL 切换到固定翼做准备，预置转换状态机。

---

## 完整场景示例

### 场景 1：QHover -> FBWA（VTOL 转固定翼）

```
时间线：
  t0: 在 QHover 悬停
      └─► in_vtol_mode() = true
      └─► 走【分支二】motors_output() + VTOL_update()

  t1: 切换到 FBWA
      └─► in_vtol_mode() = false
      └─► 走【分支一】transition->update()
          └─► 开始转换：等待空速...

  t2: 空速达到，转换完成
      └─► transition_state = DONE
      └─► 纯固定翼飞行
```

### 场景 2：FBWA -> QHover（固定翼转 VTOL）

```
时间线：
  t0: 在 FBWA 飞行
      └─► in_vtol_mode() = false
      └─► 走【分支一】transition->update()（转换已完成状态）

  t1: 切换到 QHover
      └─► in_vtol_mode() = true
      └─► 走【分支二】直接 motors_output()
          └─► 多旋翼电机立即响应，开始悬停
```

### 场景 3：Manual 模式

```
  始终在【分支一】的情况A
  └─► 多旋翼电机关闭
  └─► 纯手动固定翼飞行
```

---

## 总结图

```
┌────────────────────────────────────┐
│      QuadPlane::update()           │
│   判断当前飞行大类模式              │
└──────────────┬─────────────────────┘
               │
    ┌──────────┴──────────┐
    ▼                     ▼
┌─────────────┐     ┌─────────────┐
│  固定翼模式   │     │   VTOL模式   │
│ (分支一)     │     │  (分支二)    │
└──────┬──────┘     └──────┬──────┘
       │                   │
   ┌───┴───┐           ┌───┘
   ▼       ▼           ▼
 手动     自动      motors_output()
 模式    模式      (多旋翼飞行)
  │        │           │
  │   transition    transition
  │      ->update()   ->VTOL_update()
  │    (管理转换)    (准备下次转换)
  ▼
电机关闭
(纯固定翼)
```

**核心思想**：
- **VTOL 模式**：多旋翼电机工作，固定翼舵面辅助
- **固定翼自动模式**：可能需要转换，用 `transition` 状态机管理
- **固定翼手动模式**：多旋翼完全关闭，纯固定翼飞行
