# ArduPilot Servo Function 执行逻辑解析

## 核心架构：Function-Centric（以功能为中心）

ArduPilot 的设计非常巧妙——**代码不按通道号操作，而是按功能操作**。物理通道和功能之间的映射完全由参数决定。

---

## 一、功能定义

所有 servo function 定义在 `libraries/SRV_Channel/SRV_Channel.h:46-223`：

```cpp
typedef enum {
    k_GPIO       = -1,   // 用作 GPIO
    k_none       = 0,    // 通用 PWM，可被 Lua 脚本控制
    k_manual     = 1,    // RC 直通
    k_flap       = 2,
    k_aileron    = 4,
    k_elevator   = 19,
    k_rudder     = 21,
    k_throttle   = 70,
    k_motor1     = 33,   // 电机1
    // ... 一直到 motor32
} Function;
```

---

## 二、核心数据结构

在 `libraries/SRV_Channel/SRV_Channel.h:685-691`，有一个**以功能为索引**的静态数组：

```cpp
static struct srv_function {
    SRV_Channel::servo_mask_t channel_mask;  // 哪些物理通道配置了这个功能
    float output_scaled;                      // 该功能的输出值
} functions[SRV_Channel::k_nr_aux_servo_functions];
```

这意味着：
- `functions[k_aileron].output_scaled` 存储的是副翼输出值
- `functions[k_aileron].channel_mask` 是位掩码，标记哪些物理通道被配置为副翼

---

## 三、功能到通道的映射建立

在 `libraries/SRV_Channel/SRV_Channel_aux.cpp:222-246`：

```cpp
void SRV_Channels::update_aux_servo_function(void) {
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const uint16_t function = channels[i].function.get();
        functions[function].channel_mask |= 1U<<i;  // 建立映射
    }
}
```

---

## 四、代码如何"区分"不同功能

### 方式一：直接按功能写值（最常用）

飞行器代码（Plane、Copter 等）直接按功能枚举写值，不关心物理通道：

```cpp
// ArduPlane/servos.cpp
SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron_value);
SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator_value);
```

`set_output_scaled` 的实现（`libraries/SRV_Channel/SRV_Channel_aux.cpp:617-623`）：

```cpp
void SRV_Channels::set_output_scaled(SRV_Channel::Function function, float value) {
    if (SRV_Channel::valid_function(function)) {
        functions[function].output_scaled = value;
    }
}
```

### 方式二：switch-case 按功能分类处理

在需要特殊处理的地方，用 switch 判断功能类型：

**判断是否是电机**（`libraries/SRV_Channel/SRV_Channel.cpp:317-322`）：

```cpp
bool SRV_Channel::is_motor(Function function) {
    return ((function >= k_motor1 && function <= k_motor8) ||
            (function >= k_motor9 && function <= k_motor12) ||
            (function >= k_motor13 && function <= k_motor32));
}
```

**判断是否需要紧急停止**（`libraries/SRV_Channel/SRV_Channel.cpp:325-354`）：

```cpp
bool SRV_Channel::should_e_stop(Function function) {
    switch (function) {
    case Function::k_heli_rsc:
    case Function::k_motor1:
    case Function::k_motor2:
    // ... 所有电机/油门功能
        return true;
    default:
        return false;
    }
}
```

**按功能设置输出范围**（`libraries/SRV_Channel/SRV_Channel_aux.cpp:134-219`）：

```cpp
void SRV_Channel::aux_servo_function_setup(void) {
    switch (function.get()) {
    case k_flap:
    case k_flap_auto:
        set_range(100);      // 0-100% 范围
        break;
    case k_aileron:
    case k_elevator:
    case k_rudder:
        set_angle(4500);     // ±45度 角度范围
        break;
    case k_throttle:
        set_range(100);
        break;
    // ...
    }
}
```

---

## 五、从功能值到物理输出的完整流程

```
┌─────────────────────────────────────────┐
│  Plane/Copter 代码                      │
│  set_output_scaled(k_aileron, value)   │
└─────────────┬───────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│  functions[k_aileron].output_scaled     │
│  （功能为中心的值存储）                  │
└─────────────┬───────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│  calc_pwm()                             │
│  遍历每个物理通道：                      │
│  如果通道配置为 k_aileron，             │
│  就读取 functions[k_aileron].output_scaled │
│  转换为 PWM 值                          │
└─────────────┬───────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│  output_ch_all()                        │
│  对每个通道调用 hal.rcout->write()      │
│  特殊处理：k_manual/k_rcin* 做 RC 直通   │
└─────────────────────────────────────────┘
```

---

## 六、具体例子走查

### 假设场景

参数配置：
- `SERVO1_FUNCTION = 4` （k_aileron，副翼）
- `SERVO2_FUNCTION = 19` （k_elevator，升降舵）
- `SERVO3_FUNCTION = 70` （k_throttle，油门）

### 第一步：启动时建立映射

代码在 `SRV_Channel_aux.cpp:222-246`：

```cpp
void SRV_Channels::update_aux_servo_function(void) {
    // 先清空
    for (uint16_t i = 0; i < SRV_Channel::k_nr_aux_servo_functions; i++) {
        functions[i].channel_mask = 0;   // ← 所有功能的 channel_mask 清零
    }

    // 遍历每个物理通道
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const uint16_t function = channels[i].function.get();  // 读取该通道的功能值
        functions[function].channel_mask |= 1U<<i;  // ← 关键！把物理通道号登记到对应功能里
    }
}
```

执行完后，内存里的状态：

```
functions[4]  (k_aileron)   .channel_mask = 0b0001  ← 物理通道1
functions[19] (k_elevator)  .channel_mask = 0b0010  ← 物理通道2
functions[70] (k_throttle)  .channel_mask = 0b0100  ← 物理通道3
```

**这就是映射表。**

### 第二步：Plane 代码输出副翼值

在 `ArduPlane/servos.cpp` 某处：

```cpp
SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0.5);  // 副翼打一半
```

进入 `SRV_Channel_aux.cpp:617-623`：

```cpp
void SRV_Channels::set_output_scaled(SRV_Channel::Function function, float value) {
    if (SRV_Channel::valid_function(function)) {
        functions[function].output_scaled = value;   // ← 直接写！
    }
}
```

这里 `function = 4`（k_aileron），所以：

```
functions[4].output_scaled = 0.5
```

**注意：这里完全没有碰物理通道1，只写了功能数组。**

### 第三步：计算 PWM

在 `SRV_Channels.cpp:401-427`：

```cpp
void SRV_Channels::calc_pwm(void) {
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {   // ← 遍历物理通道
        if (channels[i].valid_function()) {
            // 关键：用物理通道的功能值，去功能数组里查对应的 output_scaled
            channels[i].calc_pwm(functions[channels[i].function.get()].output_scaled);
        }
    }
}
```

循环执行：

| i | channels[i].function | 查 functions[?].output_scaled | 结果 |
|---|----------------------|-------------------------------|------|
| 0 | 4 (k_aileron) | functions[4].output_scaled = **0.5** | 通道1得到 0.5 |
| 1 | 19 (k_elevator) | functions[19].output_scaled = ? | 通道2得到升降舵值 |
| 2 | 70 (k_throttle) | functions[70].output_scaled = ? | 通道3得到油门值 |

**这就是"反向查找"：物理通道知道自己的功能号，去功能数组里取对应的值。**

---

## 七、流程图（简化版）

```
启动时建立映射：
┌─────────────┐     ┌─────────────────┐
│ SERVO1=4    │────→│ functions[4]    │
│ SERVO2=19   │────→│ functions[19]   │   ← 功能 → 物理通道的映射表
│ SERVO3=70   │────→│ functions[70]   │
└─────────────┘     └─────────────────┘

运行时写值（Plane代码）：
SRV_Channels::set_output_scaled(k_aileron, 0.5)
                           │
                           ▼
              functions[4].output_scaled = 0.5
                           ↑
                           这里只操作功能数组，不碰物理通道

计算PWM时（calc_pwm）：
物理通道0 ──→ function=4 ──→ 查 functions[4].output_scaled ──→ 得到 0.5
物理通道1 ──→ function=19 ─→ 查 functions[19].output_scaled ─→ 得到升降舵值
物理通道2 ──→ function=70 ─→ 查 functions[70].output_scaled ─→ 得到油门值
```

---

## 八、一句话总结

> **写的时候按功能写，读的时候物理通道按自己的功能号去查。**

功能数组是"中央仓库"，Plane/Copter 代码只管往仓库里放值，不管谁来接。`calc_pwm` 时每个物理通道来仓库里按自己的功能号取货。

这就是为什么你可以把副翼接到任意 SERVO 口——因为映射是在启动时动态建立的，代码逻辑完全不硬编码通道号。

---

## 九、相关源文件

| 文件 | 作用 |
|------|------|
| `libraries/SRV_Channel/SRV_Channel.h` | 功能枚举定义、核心数据结构 |
| `libraries/SRV_Channel/SRV_Channel_aux.cpp` | 映射建立、输出设置、特殊功能处理 |
| `libraries/SRV_Channel/SRV_Channels.cpp` | calc_pwm 主流程 |
| `libraries/SRV_Channel/SRV_Channel.cpp` | 单通道 PWM 计算、功能分类判断 |
| `ArduPlane/servos.cpp` | Plane 飞行器输出示例 |
