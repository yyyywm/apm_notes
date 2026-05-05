# ArduPilot 活塞发动机控制详解：化油器 vs 电喷（EFI）

## 核心架构概览

ArduPilot 对两种发动机的控制逻辑由两个独立库分管：

- **化油器**：`AP_ICEngine`（`libraries/AP_ICEngine/`）—— 直接控制舵机输出
- **电喷**：`AP_EFI`（`libraries/AP_EFI/`）—— 通过通信协议与 ECU 交互

---

## 1. 化油器发动机控制逻辑

### 硬件原理

化油器是纯机械装置，通过文丘里效应将燃油与空气按固定比例混合。油门控制完全依赖**油门舵机**机械连接化油器的节气门阀（throttle valve）。没有 ECU，没有任何反馈闭环。

```
油门舵机 → 机械连杆 → 化油器节气门 → 混合气比例固定（由化油器设计决定）
```

### ArduPilot 控制逻辑

#### 关键文件

| 文件 | 作用 |
|------|------|
| `libraries/AP_ICEngine/AP_ICEngine.cpp` | 核心控制逻辑 |
| `libraries/AP_ICEngine/AP_ICEngine.h` | 参数定义、状态枚举 |
| `libraries/SRV_Channel/SRV_Channel.h` | 舵机通道功能定义 |

#### 通道映射（SRV_Channel）

```
SRV_Channel::k_throttle   (功能70) → 油门舵机 PWM输出 (1000~2000μs)
SRV_Channel::k_ignition   (功能67) → 点火信号 PWM输出
SRV_Channel::k_starter    (功能69) → 启动机信号 PWM输出
SRV_Channel::k_choke      (功能68) → 风门(Choke) PWM输出 （代码中标记为"not used"）
```

初始化代码（`AP_ICEngine.cpp:212-217`）：

```cpp
SRV_Channels::set_range(SRV_Channel::k_starter, 1);      // range type = 二值开关
SRV_Channels::set_range(SRV_Channel::k_ignition, 1);
SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_starter, 1000, 2000);
SRV_Channels::set_output_min_max_defaults(SRV_Channel::k_ignition, 1000, 2000);
```

#### 启动控制状态机

`ICE_State` 枚举定义于 `AP_ICEngine.h:52-59`：

```
ICE_OFF → ICE_START_HEIGHT_DELAY → ICE_START_DELAY → ICE_STARTING → ICE_RUNNING
         (等待高度条件)           (starter_delay秒)    (starter_time秒)  (检测RPM)
```

**启动流程**（`AP_ICEngine.cpp:300-513`）：

1. `ICE_OFF`：当 RC 辅助开关 HIGH (≥1700 PWM) → 进入 `ICE_START_DELAY`
2. `ICE_START_DELAY`：等待 `starter_delay` 秒（默认2秒）→ 进入 `ICE_STARTING`
3. `ICE_STARTING`：
   - `set_ignition(true)` — 接通点火
   - `set_starter(true)` — 接通启动机
   - 持续 `starter_time` 秒（默认3秒）
4. `ICE_RUNNING`：检测到 RPM > `rpm_threshold`（默认100）→ 认为发动机成功启动

#### 点火与启动机输出（`AP_ICEngine.cpp:708-738`）

```cpp
void AP_ICEngine::set_ignition(bool on) {
    SRV_Channels::set_output_scaled(SRV_Channel::k_ignition, on ? 1.0 : 0.0);
    AP_Relay::set(FUNCTION::IGNITION, on);  // 同时控制继电器
}

void AP_ICEngine::set_starter(bool on) {
    SRV_Channels::set_output_scaled(SRV_Channel::k_starter, on ? 1.0 : 0.0);
    AP_Relay::set(FUNCTION::ICE_STARTER, on);  // 可选继电器控制
}
```

**注意**：ArduPilot 只负责**接通/断开**启动机和点火电路，不控制启动机的时序或脉冲方式。启动机的通断完全由 `starter_time` 参数决定（3秒后自动断开）。

#### 油门控制

油门通过 `SRV_Channel::k_throttle` 直接输出 PWM 信号到油门舵机。`AP_ICEngine` 通过 `throttle_override()` 函数（`AP_ICEngine.cpp:524-589`）在以下场景干预油门：

- **启动时**：强制将油门设为 `start_percent`（默认5%）
- **怠速**：维持 `idle_percent`（默认0%，可配置）
- **RPM超转**：Redline Governor 限制油门

```cpp
bool AP_ICEngine::throttle_override(float &percentage, const float base_throttle) {
    if (state == ICE_STARTING || state == ICE_START_DELAY) {
        percentage = start_percent.get();  // 强制启动油门
        return true;
    }
    if (state == ICE_RUNNING && min_throttle_pct > percentage) {
        percentage = min_throttle_pct;   // 强制怠速油门
        return true;
    }
    // Redline Governor ...
}
```

#### RC 触发方式

在 ArduPlane 中，通过 `AUX_FUNC::ICE_START_STOP` 辅助功能（`RC_Channel_Plane.cpp:487-489`）：

- RC通道 PWM ≥ 1700 → 启动发动机
- RC通道 PWM ≤ 1300 → 停止发动机

#### 关键参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `ICE_ENABLE` | 使能 ICEngine 控制 | 0 (禁用) |
| `ICE_START_PCT` | 启动时油门百分比 | 5% |
| `ICE_IDLE_PCT` | 怠速时最小油门百分比 | 0% |
| `ICE_STARTER_TIME` | 启动机运行时间(秒) | 3s |
| `ICE_START_DELAY` | 启动尝试间隔(秒) | 2s |
| `ICE_RPM_THRESH` | 认为发动机运转的RPM阈值 | 100 |
| `ICE_RPM_CHAN` | RPM传感器实例 | 0 (无) |
| `ICE_IDLE_RPM` | 怠速Governor目标RPM (-1禁用) | -1 |
| `ICE_REDLINE_RPM` | 红线RPM (0禁用) | 0 |

---

## 2. 电喷发动机控制逻辑

### 硬件原理

EFI 系统有独立的 ECU，ECU 根据**节气门位置（TPS）**、进气歧管压力（MAP）、水温（ECT）、氧传感器（O2）等传感器自行计算喷油量和点火提前角。ArduPilot 只需告诉 ECU "要多少油门"，ECU 自己决定喷多少油、怎么点火。

```
ArduPilot throttle PWM → ECU throttle input → ECU自行计算喷油/点火 → 发动机
         ↑                                              ↑
    只发油门命令                               ECU完全自主控制
```

### ArduPilot 控制逻辑

#### 关键文件

| 文件 | 作用 |
|------|------|
| `libraries/AP_EFI/AP_EFI.cpp` | 前端聚合、状态管理 |
| `libraries/AP_EFI/AP_EFI_State.h` | 发动机状态数据结构 |
| `libraries/AP_EFI/AP_EFI_Backend.h` | 后端抽象基类 |
| `libraries/AP_EFI/AP_EFI_Serial_MS.cpp` | MegaSquirt 串口协议后端 |
| `libraries/AP_EFI/AP_EFI_DroneCAN.cpp` | DroneCAN/UAVCAN 协议后端 |
| `libraries/AP_EFI/AP_EFI_Serial_Hirth.cpp` | Hirth 串口协议后端 |
| `libraries/AP_EFI/AP_EFI_Currawong_ECU.cpp` | Currawong CAN 协议后端 |
| `libraries/AP_EFI/AP_EFI_MAV.cpp` | MAVLink 转发后端 |

#### EFI_TYPE 参数（`AP_EFI.cpp:48`）

```cpp
enum class Type : uint8_t {
    NONE = 0,
    MegaSquirt = 1,    // 串口
    NWPMU = 2,         // 串口
    Lutan = 3,         // 串口
    LOWEHEISER = 4,    // 串口/CAN
    DroneCAN = 5,      // CAN总线 (UAVCAN)
    CurrawongECU = 6,  // CAN总线
    SCRIPTING = 7,     // Lua脚本
    Hirth = 8,         // 串口
    MAV = 9,           // MAVLink转发
};
```

#### 通信协议流程（以 Hirth 为例）

Hirth 协议通过串口同时完成**发送油门**和**接收状态**（`AP_EFI_Serial_Hirth.cpp:153-194`）：

```cpp
void AP_EFI_Serial_Hirth::send_request() {
    const uint16_t new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if ((new_throttle != last_throttle) || (now - last_req_send_throttle_ms > 500)) {
        // 仅在 armed 状态发送非零油门
        bool allow_throttle = hal.util->get_soft_armed();
        if (allow_throttle) {
            send_target_values(new_throttle);  // 发油门命令到ECU
        } else {
            send_target_values(0);             // disarmed → 油门=0
        }
    } else {
        send_request_status();  // 请求ECU状态
    }
}
```

#### ArduPilot ← ECU（数据流）

ECU 周期性地向 ArduPilot 推送状态，ArduPilot 记录在 `EFI_State` 结构体中：

```cpp
struct EFI_State {
    uint32_t engine_speed_rpm;              // 转速
    uint8_t engine_load_percent;             // 发动机负载
    uint8_t throttle_position_percent;       // 节气门位置（ECU反馈）
    float coolant_temperature;               // 水温
    float intake_manifold_temperature;       // 进气温度
    float intake_manifold_pressure_kpa;      // 进气歧管压力
    float fuel_pressure;                     // 燃油压力
    float fuel_consumption_rate_cm3pm;       // 燃油消耗率
    Cylinder_Status cylinder_status;         // 每缸状态
    Engine_State engine_state;               // STOPPED/STARTING/RUNNING/FAULT
    // ... 更多状态
};
```

`throttle_position_percent` 是 ECU 返回的**实际节气门开度**，而非 ArduPilot 发出的指令值。`throttle_out` 是 ArduPilot 发出的油门指令值。

#### 自动怠速

当 `idle_rpm > 0` 时启用（`AP_ICEngine.cpp:635-702`）：

```cpp
void AP_ICEngine::update_idle_governor(int8_t &min_throttle) {
    float rpmv;
    ap_rpm->get_rpm(rpm_instance-1, rpmv);
    int32_t error = idle_rpm - rpmv;  // 误差
    if (error > 0) {  // 低于目标转速
        idle_governor_integrator += idle_setpoint_step;  // 增加怠速油门
    } else {
        idle_governor_integrator -= idle_setpoint_step;  // 减少怠速油门
    }
    min_throttle = roundf(idle_governor_integrator);
}
```

这个怠速 governor 是在 `AP_ICEngine` 中实现的，它调整的是 `min_throttle_pct`——最终影响的是 `k_throttle` 输出的 PWM 数值，而 ECU 根据这个油门指令决定怠速喷油量。

#### 启动时序

EFI 系统的启动完全由 ECU 自主管理。ArduPilot 通过 `AP_ICEngine` 控制：

1. `set_ignition(true/false)` — 命令 ECU 点火/断火
2. `set_starter(true/false)` — 控制启动机继电器
3. 油门值通过 `k_throttle` 输出 → 传给 ECU → ECU 决定启动喷油策略

ArduPilot **不参与** ECU 的启动时序控制（喷油量、启动加浓策略等均由 ECU 固件决定）。

#### 保护逻辑

| 保护功能 | 实现位置 | 机制 |
|---------|---------|------|
| RPM超转 | `AP_ICEngine.cpp:461-479` | Redline Governor → 限制 `k_throttle` 输出 |
| 怠速 RPM 稳定 | `AP_ICEngine.cpp:635-702` | Idle Governor → 调整 `min_throttle_pct` |
| 燃油压力监控 | `EFI_State.fuel_pressure_status` | ECU 内置，ArduPilot 只读取状态 |
| 水温/EGT 监控 | `EFI_State.temperature_status` | ECU 内置，ArduPilot 只读取状态 |
| 点火电压监控 | `EFI_State.ignition_voltage` | ECU 检测，ArduPilot 只读取状态 |

---

## 3. 启动机控制逻辑

### 化油器

**直接 PWM 输出控制**：ArduPilot 通过 `SRV_Channel::k_starter` 输出 PWM（默认2000μs=启动，1000μs=停止），直接驱动启动机继电器或电调。

```cpp
void AP_ICEngine::set_starter(bool on) {
    SRV_Channels::set_output_scaled(SRV_Channel::k_starter, on ? 1.0 : 0.0);
    // 1.0 = 2000μs (PWM_STRT_ON), 0.0 = 1000μs (PWM_STRT_OFF)
}
```

启动时间由 `starter_time` 参数控制（默认3秒），到达后自动关闭。

### EFI

启动机控制逻辑**相同**——同样通过 `k_starter` PWM 输出。唯一的区别是：对于 EFI 发动机，启动机的通断也同时作为 ECU 的"启动请求"信号（ECU 感知到启动机通电后自行进入启动模式）。

---

## 4. 风门（Choke）控制

### 化油器 — 需要 Choke

冷启动时化油器需要加浓混合气（choke 关闭减少空气进入）。ArduPilot 中：

- `SRV_Channel::k_choke`（功能68）**存在但标记为 not used**（`SRV_Channel.h:112`）
- SITL 模拟器中有 choke 实现（`SIM_ICEngine.cpp:70-73`）：choke 开启时发动机只能运转1秒后熄火（模拟过rich窒息）

**实际物理连接**：Choke 通常由**舵机机械控制**或**手拉绳控制**，ArduPilot 没有实现自动化 choke 控制。冷启动需要飞手手动拉 choke 绳。

### EFI — 不需要 Choke

ECU 根据**水温传感器（ECT）**自动判断冷启动状态，在固件中内置冷启动加浓策略（增加喷油量、延迟点火提前角），完全不需要外部 choke 控制。

---

## 5. ECU 是否参与自主控制？

### 化油器：ArduPilot 全权控制

化油器发动机没有 ECU。ArduPilot 是唯一控制者：

- 点火：ArduPilot 直接控制 `k_ignition` PWM
- 油门：ArduPilot 直接控制 `k_throttle` PWM → 舵机 → 化油器节气门
- 启动：ArduPilot 直接控制 `k_starter` PWM

### EFI：ECU 有很大自主权

```
ArduPilot                    ECU                        发动机
    │                         │                           │
    ├── 发油门(0-100%) ──────►│ ECU解析指令               │
    │                         │                           │
    │                         ├──→ 查MAP/TPS/ECT表        │
    │                         ├──→ 计算喷油量              │
    │                         ├──→ 计算点火正时           │
    │                         ├──→ 控制喷油器驱动         │
    │                         └──→ 控制点火线圈           │
    │                         │                           │
    │◄── 返回 RPM/温度/状态 ───┘                           │
```

**ArduPilot 发指令（油门值），ECU 自主决定如何执行**：

- ArduPilot 告诉 ECU "我要50%油门"
- ECU 自行查表决定：在当前水温、进气温度、大气压力下，50%油门对应多少喷油脉宽、点火提前角是多少
- ArduPilot 只接收 ECU 上报的 RPM、温度等状态，不参与底层控制

**ArduPilot 保留的控制权**：

1. `set_ignition(on/off)` — 直接命令 ECU 点火/断火（不是通过油门）
2. `set_starter(on/off)` — 启动机继电器
3. Redline Governor — 超转时干预油门
4. Idle Governor — 怠速转速干预油门

---

## 6. 熄火（Shutdown）流程

### 熄火触发条件

熄火**不区分**发动机类型，统一由 `AP_ICEngine` 状态机处理。核心判断在于 `should_run` 何时为 `false`。

**触发条件**（`AP_ICEngine.cpp:317-328`）：

```cpp
// RC辅助开关 LOW (≤1300μs) → 熄火
if (aux_pos == RC_Channel::AuxSwitchPos::LOW) {
    should_run = false;
}

// DISABLE_IGNITION_RC_FAILSAFE 时，RC失联 → 熄火
if (option_set(Options::DISABLE_IGNITION_RC_FAILSAFE) &&
    AP_Notify::flags.failsafe_radio) {
    should_run = false;
}

// 降落伞释放 → 熄火
if (parachute->release_initiated()) {
    should_run = false;
}

// 紧急停机 → 熄火
if (SRV_Channels::get_emergency_stop()) {
    should_run = false;
}
```

### ICE_OFF 状态动作

```cpp
case ICE_OFF:
    set_ignition(false);   // 切断点火
    set_starter(false);    // 切断启动机
    starter_start_time_ms = 0;
    break;
```

### 化油器 vs EFI 熄火机制对比

| 发动机类型 | 熄火方式 | 说明 |
|-----------|---------|------|
| 化油器 | 切断点火 | 切断点火后发动机立即停转（无燃油供应 + 无火花） |
| 电喷 | 切断点火 + 油门归零 | ECU 检测到熄火请求后，自主停止喷油和点火 |

### EFI 熄火时的额外动作

除了切断点火外，`throttle_override` 在非 running 状态时会**强制油门归零**（`AP_EFI_Serial_Hirth.cpp:169-181`）：

```cpp
if (!allow_throttle) {
    send_target_values(0);  // 发油门=0 给ECU
}
```

### 熄火状态机转换

```
ICE_RUNNING ──► should_run=false ──► ICE_OFF
                                ├── set_ignition(false)
                                ├── set_starter(false)
                                └── 油门归零 (via throttle_override)
```

### 非预期熄火（Uncommanded Engine Stop）

当发动机在 RUNNING 状态下 RPM 突然跌到阈值以下，会触发 `ICE_START_DELAY` 重新尝试点火（`AP_ICEngine.cpp:419-441`）：

```cpp
case ICE_RUNNING:
    if (!should_run) {
        state = ICE_OFF;
    } else if (rpm_instance > 0) {
        float rpm_value;
        if (!AP::rpm()->get_rpm(rpm_instance-1, rpm_value) ||
            rpm_value < rpm_threshold) {
            // 发动机意外熄火 → 重新进入启动延迟
            state = ICE_START_DELAY;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Uncommanded engine stop");
        }
    }
    break;
```

### 关键结论

熄火流程中 `AP_EFI` **不参与** 熄火控制——熄火逻辑全在 `AP_ICEngine` 里。`AP_EFI` 只被动监听 ECU 上报的 `engine_state`（STOPPED/RUNNING），ArduPilot 没有主动发熄火命令给 ECU（因为没有这个协议接口）。

**熄火 = 切断点火 + 油门归零，ECU 感知到点火中断后自行停止喷油。**

---

## 7. 关键代码文件对应

| 功能 | 文件 | 关键函数/类 |
|------|------|------------|
| ICEngine 总控 | `libraries/AP_ICEngine/AP_ICEngine.cpp` | `update()`, `throttle_override()`, `set_ignition()`, `set_starter()` |
| ICEngine 参数 | `libraries/AP_ICEngine/AP_ICEngine.h` | `ICE_State` 枚举, `var_info[]` 参数 |
| EFI 前端 | `libraries/AP_EFI/AP_EFI.cpp` | `update()`, `send_mavlink_status()` |
| EFI 状态结构 | `libraries/AP_EFI/AP_EFI_State.h` | `EFI_State`, `Engine_State`, `Cylinder_Status` |
| EFI DroneCAN后端 | `libraries/AP_EFI/AP_EFI_DroneCAN.cpp` | `handle_status()` — 解析 UAVCAN 消息 |
| EFI Hirth后端 | `libraries/AP_EFI/AP_EFI_Serial_Hirth.cpp` | `send_request()`, `send_target_values()` — 发送油门到ECU |
| 油门输出 | `ArduPlane/servos.cpp:951-958` | `throttle_override()` 调用点 |
| 启动RC触发 | `ArduPlane/RC_Channel_Plane.cpp:487-489` | `ICE_START_STOP` aux function |
| DO_ENGINE_CONTROL | `AP_ICEngine.cpp:594-628` | `engine_control()` MAVLink/任务命令入口 |
| 怠速Governor | `AP_ICEngine.cpp:635-702` | `update_idle_governor()` |
| Redline Governor | `AP_ICEngine.cpp:556-578` | `redline.flag` 逻辑 |
| SRV通道定义 | `libraries/SRV_Channel/SRV_Channel.h:46-200` | `k_throttle=70`, `k_ignition=67`, `k_starter=69`, `k_choke=68` |
| SITL模拟 | `libraries/SITL/SIM_ICEngine.cpp` | 化油器发动机模拟（含choke逻辑） |

---

## 7. 数据流总结

### 化油器 (AP_ICEngine)

```
RC Aux Switch ──► ICE_START_STOP ──► 状态机 ──► set_ignition()
                                        ├──► set_starter()
                                        └──► throttle_override()
                                              k_throttle PWM ──► 油门舵机
```

### 电喷 (AP_EFI + AP_ICEngine)

```
RC Aux ──► ICE_START_STOP ──► AP_ICEngine ──► set_ignition() ──► ECU
                                          ├──► set_starter()
                                          └──► throttle_override()
                                                         │
                                                         ▼
                              SRV_Channel::k_throttle ──► ECU油门输入
                                                         │
                                                         ▼
  ECU ──(DroneCAN/Serial/MAV)──► AP_EFI_Backend ──► EFI_State
                                    ├── RPM
                                    ├── 温度
                                    ├── 压力
                                    └── throttle_out (ECU实际开度)
```

---

## 8. MAVLink 命令接口

两种发动机系统都通过 `MAV_CMD_DO_ENGINE_CONTROL` 命令控制：

```cpp
// ArduPlane/GCS_MAVLink_Plane.cpp:799-804
case MAV_CMD_DO_ENGINE_CONTROL:
    plane.g2.ice_control.engine_control(packet.param1,     // start_control
                                        packet.param2,     // cold_start
                                        packet.param3,     // height_delay_cm
                                        packet.param4);    // flags
    return MAV_RESULT_ACCEPTED;
```

| 参数 | 说明 |
|------|------|
| param1 | 启动控制 (>0启动, ≤0停止) |
| param2 | 冷启动标志（传递给ECU） |
| param3 | 高度延迟（厘米，到达高度后才启动） |
| param4 | 选项标志（如 `ALLOW_START_WHILE_DISARMED`） |

---

*文档生成时间：2026-04-13*
