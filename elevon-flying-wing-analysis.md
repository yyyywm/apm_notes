# ArduPilot 纯固定翼飞翼/三角翼（Elevon）源码分析

## 1. 机型标识与相关参数

ArduPlane **不通过 FRAME_CLASS** 来区分飞翼/三角翼，而是通过以下参数组合来配置：

### 关键参数

| 参数 | 文件位置 | 说明 |
|------|---------|------|
| **DSPOILER_OPTS** | [Parameters.cpp:1127](../ArduPlane/Parameters.cpp#L1127) | Bit0 = `FLYWING` 标志 |
| **MIXING_GAIN** | [Parameters.cpp:603](../ArduPlane/Parameters.cpp#L603) | 混控增益，默认 0.5 |
| **MIXING_OFFSET** | [Parameters.cpp:618](../ArduPlane/Parameters.cpp#L618) | 混控偏置 (-100~100 %) |
| **KFF_RDDRMIX** | [Parameters.cpp:54](../ArduPlane/Parameters.cpp#L54) | 方向舵混迷系数 |
| **RUDDER_ONLY** | [Parameters.cpp:610](../ArduPlane/Parameters.cpp#L610) | 仅方向舵机型标志 |

### CrowFlapOptions 定义
定义于 [defines.h:167-171](../ArduPlane/defines.h#L167):

```cpp
enum CrowFlapOptions {
    FLYINGWING       = (1 << 0),   // 飞翼/三角翼标志
    FULLSPAN         = (1 << 1),   // 全翼展副翼
    PROGRESSIVE_CROW = (1 << 2),   // 渐进式 crows foot
};
```

### 飞翼识别方式
在 [servos.cpp:227](../ArduPlane/servos.cpp#L227):

```cpp
const bool flying_wing = (bitmask & CrowFlapOptions::FLYWINGWING) != 0;
```

---

## 2. 方向舵通道的禁用逻辑

### 方式一：RUDDER_ONLY 参数
在 [radio.cpp:12-21](../ArduPlane/radio.cpp#L12):

```cpp
void Plane::set_control_channels(void)
{
    if (g.rudder_only) {
        // 在 rudder only 模式下，roll 和 yaw 通道相同
        channel_roll = &rc().get_yaw_channel();
    } else {
        channel_roll = &rc().get_roll_channel();
    }
    channel_pitch    = &rc().get_pitch_channel();
    channel_throttle = &rc().get_throttle_channel();
    channel_rudder   = &rc().get_yaw_channel();
    // ...
}
```

### 方式二：若无方向舵物理通道
在 [radio.cpp:166-186](../ArduPlane/radio.cpp#L166):

```cpp
int16_t Plane::rudder_input(void)
{
    if (g.rudder_only != 0) {
        return 0;  // 丢弃方向舵输入
    }
    // ...
    if (stick_mixing_enabled()) {
        return channel_rudder->get_control_in();
    }
    return 0;
}
```

### 关键：当 k_rudder 功能未分配时
方向舵输出到 `SRV_Channel::k_rudder`，如果该功能未通过 `SERVO*_FUNCTION` 参数分配给任何物理通道，则方向舵控制信号不会输出到任何实际舵机。这是飞翼机型"禁用方向舵"的实际机制。

---

## 3. 无方向舵时的偏航（Yaw）控制与协调转弯

### 协调转弯计算
定义于 [Attitude.cpp:527-569](../ArduPlane/Attitude.cpp#L527):

```cpp
int16_t Plane::calc_nav_yaw_coordinated()
{
    const float speed_scaler = get_speed_scaler();
    int16_t rudder_in = rudder_input();
    int16_t commanded_rudder;

    commanded_rudder = yawController.get_servo_out(speed_scaler, disable_integrator);

    // 添加来自副翼的方向舵混迷（逆偏航补偿）
    commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
    commanded_rudder += rudder_in;

    return constrain_int16(commanded_rudder, -4500, 4500);
}
```

### 逆偏航补偿原理
当副翼产生滚转时，飞机会产生天然偏航（逆偏航）。`KFF_RDDRMIX` 参数将副翼信号叠加到方向舵上，补偿这一效应。对于无方向舵的飞翼，这部分逻辑仍然计算，但如果没有物理方向舵通道，信号不会输出。

### 方向舵输出
在 [Attitude.cpp:405-419](../ArduPlane/Attitude.cpp#L405):

```cpp
if (!ground_steering) {
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_output);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder_output);
} else if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_output);
    // ...
}
```

---

## 4. Elevon 混控的完整计算公式与代码实现

### 核心混控公式
定义于 [servos.cpp:176-196](../ArduPlane/servos.cpp#L176):

```cpp
void Plane::channel_function_mixer(
    SRV_Channel::Function func1_in,  // 通道1输入（如 k_aileron）
    SRV_Channel::Function func2_in,  // 通道2输入（如 k_elevator）
    SRV_Channel::Function func1_out, // 通道1输出（如 k_elevon_left）
    SRV_Channel::Function func2_out  // 通道2输出（如 k_elevon_right）
) const
{
    float in1 = SRV_Channels::get_output_scaled(func1_in);
    float in2 = SRV_Channels::get_output_scaled(func2_in);

    // 应用 MIXING_OFFSET
    if (g.mixing_offset < 0) {
        in2 *= (100 - g.mixing_offset) * 0.01;
    } else if (g.mixing_offset > 0) {
        in1 *= (100 + g.mixing_offset) * 0.01;
    }

    // 核心混控公式：
    // elevon_left  = (elevator - aileron) * MIXING_GAIN
    // elevon_right = (elevator + aileron) * MIXING_GAIN
    float out1 = constrain_float((in2 - in1) * g.mixing_gain, -4500, 4500);
    float out2 = constrain_float((in2 + in1) * g.mixing_gain, -4500, 4500);

    SRV_Channels::set_output_scaled(func1_out, out1);
    SRV_Channels::set_output_scaled(func2_out, out2);
}
```

### 调用位置
在 [servos.cpp:1027](../ArduPlane/servos.cpp#L1027) 的 `servos_output()` 函数中：

```cpp
// elevon 混控
channel_function_mixer(SRV_Channel::k_aileron, SRV_Channel::k_elevator,
                       SRV_Channel::k_elevon_left, SRV_Channel::k_elevon_right);
```

### 混控公式分解

| 操作 | elevon_left | elevon_right |
|------|------------|--------------|
| 俯仰(+) | +elevator | +elevator |
| 俯仰(-) | -elevator | -elevator |
| 右滚转(+) | -aileron | +aileron |
| 左滚转(-) | +aileron | -aileron |

**结果**：
- 升降：两个舵面同向偏转
- 滚转：两个舵面反向偏转（差动）

---

## 5. 滚转、俯仰指令如何叠加输出到左右两个舵面

### 数值叠加逻辑

以标准配置为例（elevon_left → 左侧舵面，elevon_right → 右侧舵面）：

```
左侧舵面输出 = elevator * MIXING_GAIN - aileron * MIXING_GAIN + trim_left
右侧舵面输出 = elevator * MIXING_GAIN + aileron * MIXING_GAIN + trim_right
```

### 自动微调（Auto Trim）中的处理
在 [servos.cpp:1117-1118](../ArduPlane/servos.cpp#L1117):

```cpp
g2.servo_channels.adjust_trim(SRV_Channel::k_elevon_left,  pitch_I - roll_I);
g2.servo_channels.adjust_trim(SRV_Channel::k_elevon_right, pitch_I + roll_I);
```

这确保在自动驾驶仪控制时，左右 elevon 的 trim 值能独立调整。

---

## 6. 与传统固定翼的代码差异

### 传统固定翼（有独立副翼、升降舵、方向舵）

```
独立通道：
- k_aileron (副翼) → 直接输出到副翼舵机
- k_elevator (升降舵) → 直接输出到升降舵舵机
- k_rudder (方向舵) → 直接输出到方向舵舵机
```

### 飞翼/三角翼（只有 elevons）

```
 elevon 混控后：
- k_elevon_left  = (k_elevator - k_aileron) * MIXING_GAIN
- k_elevon_right = (k_elevator + k_aileron) * MIXING_GAIN
- k_rudder 可能未分配（无物理方向舵）
```

### Differential Spoiler（裂根式方向舵）额外混控
定义于 [servos.cpp:224-314](../ArduPlane/servos.cpp#L224)，当 `FLYWING` 标志设置时：

```cpp
void Plane::dspoiler_update(void)
{
    const bool flying_wing = (bitmask & CrowFlapOptions::FLYWINGWING) != 0;

    if (flying_wing) {
        // 从已计算的 elevon 输出开始
        elevon_left = SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left);
        elevon_right = SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right);
    } else {
        // 使用普通副翼
        elevon_left = -aileron;
        elevon_right = aileron;
    }

    // 添加方向舵产生的偏航（通过差动副翼实现）
    if (rudder > 0) {
        // 方向舵偏右 → 右翼上扬，左翼下压
        dspoiler_outer_right = constrain_float(dspoiler_outer_right + rudder, -4500, 4500);
        dspoiler_inner_right = constrain_float(dspoiler_inner_right - rudder, -4500, 4500);
    } else {
        dspoiler_outer_left = constrain_float(dspoiler_outer_left - rudder, -4500, 4500);
        dspoiler_inner_left = constrain_float(dspoiler_inner_left + rudder, -4500, 4500);
    }
}
```

---

## 7. 构型专属参数及其定义

### 参数定义文件
`ArduPlane/Parameters.cpp` 和 `ArduPlane/Parameters.h`

### 专属参数列表

| 参数名 | 类型 | 默认值 | 作用 |
|--------|------|--------|------|
| **DSPOILER_OPTS** | int8 | 3 | Bitmask：`FLYWING`(1), `FULLSPAN`(2), `PROGRESSIVE_CROW`(4) |
| **MIXING_GAIN** | float | 0.5 | elevon/vtail 混控增益 |
| **MIXING_OFFSET** | float | 0 | 混控偏置（d%），调整左右舵面响应差异 |
| **KFF_RDDRMIX** | float | 0.5 (RUDDER_MIX) | 副翼→方向舵混迷系数，补偿逆偏航 |
| **DSPOILR_RUD_RATE** | int16 | 100 | 方向舵对差动副翼的影响率 (%) |
| **DSPOILER_CROW_W1** | int16 | 0 | 外侧 crows foot 权重 (0-100) |
| **DSPOILER_CROW_W2** | int16 | 0 | 内侧 crows foot 权重 (0-100) |
| **DSPOILER_AILMTCH** | int16 | 100 | 副翼匹配系数，限制内侧副翼下偏范围 |
| **RUDDER_ONLY** | int8 | 0 | 方向舵专用机型标志 |

### 参数在代码中的使用

**MIXING_GAIN** 的应用（[servos.cpp:192-193](../ArduPlane/servos.cpp#L192)）：
```cpp
float out1 = constrain_float((in2 - in1) * g.mixing_gain, -4500, 4500);
float out2 = constrain_float((in2 + in1) * g.mixing_gain, -4500, 4500);
```

**KFF_RUDDER_MIX** 的应用（[Attitude.cpp:557](../ArduPlane/Attitude.cpp#L557)）：
```cpp
commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
```

---

## 总结：飞翼/三角翼配置要点

1. **设置 `DSPOILER_OPTS` Bit0 = 1** 启用 FLYWING 模式
2. **配置 `SERVO*n_FUNCTION`** 将两个通道设为 `k_elevon_left` 和 `k_elevon_right`
3. **调整 `MIXING_GAIN`**（默认0.5）控制舵面灵敏度
4. **使用 `MIXING_OFFSET`** 补偿左右舵面机械差异
5. **若无物理方向舵**，确保 `k_rudder` 未分配功能，方向舵控制信号不会输出
6. **协调转弯**仍通过 `calc_nav_yaw_coordinated()` 计算，`KFF_RDDRMIX` 提供逆偏航补偿
