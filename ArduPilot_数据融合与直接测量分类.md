# ArduPilot 数据融合与直接测量分类

> 本文档系统梳理 ArduPilot 中各类飞行数据的来源，明确区分哪些是**经过 EKF/AHRS 融合后**的数据，哪些是**直接从传感器测量**的数据。

---

## 一、经过 EKF/AHRS 融合后才显示/使用的数据

这些数据通过 `AP_AHRS::xxx()` 接口获取，底层由 EKF3/EKF2/DCM 进行状态估计和融合。

| 数据 | 显示来源 | 融合说明 |
|-----|---------|---------|
| **位置 (lat/lng/alt)** | `GLOBAL_POSITION_INT` | EKF3 融合 GPS + IMU + 气压计 + 其他传感器 |
| **水平速度 (vx/vy)** | `GLOBAL_POSITION_INT` / `LOCAL_POSITION_NED` | EKF3 融合多源数据 |
| **垂直速度 (vz/climb_rate)** | `VFR_HUD` / `GLOBAL_POSITION_INT` | EKF3 融合，失败时回退到气压计 |
| **姿态 (roll/pitch/yaw)** | `ATTITUDE` | EKF3 融合 IMU + 磁力计 + GPS |
| **航向 (heading)** | `VFR_HUD` / `GLOBAL_POSITION_INT` | EKF3 融合 |
| **地速 (groundspeed)** | `VFR_HUD` | EKF3 估计（融合 GPS + IMU）|
| **风 (wind)** | `WIND` | EKF3 估计（融合空速 + GPS + IMU）|
| **相对高度** | `GLOBAL_POSITION_INT.relative_alt` | EKF3 融合 |

### 关键代码路径

```cpp
// 位置获取
AP_AHRS::get_location() → NavEKF3::getPosNE() / getPosD()

// 速度获取
AP_AHRS::get_velocity_NED() → NavEKF3::getVelocityNED()

// 姿态获取
AP_AHRS::get_roll_rad() / get_pitch_rad() / get_yaw_rad() → NavEKF3::getEulerAngles()
```

### 回退机制

| 数据类型 | 主来源 | 回退 1 | 回退 2 |
|----------|--------|--------|--------|
| 位置 | EKF3 | EKF2 | DCM |
| 速度 | EKF3 | EKF2 | DCM / GPS |
| 姿态 | EKF3 | EKF2 | DCM |
| 高度 | EKF3 | 气压计互补滤波 | 纯气压计 |

---

## 二、直接从传感器测量显示的数据（未经 EKF 融合）

这些数据直接从驱动层读取，**不经过 EKF/AHRS 融合**，保持原始传感器特性。

| 数据 | 显示来源 | 说明 |
|-----|---------|------|
| **原始 GPS** | `GPS_RAW_INT` / `GPS2_RAW` | GPS 接收机直接输出，**未经任何融合** |
| **原始气压计** | `BARO` 日志 | 气压换算高度，**未经 EKF** |
| **原始 IMU** | `IMU` / `ACC` / `GYR` 日志 | 原始角速度/加速度，**未经 EKF** |
| **原始磁力计** | `MAG` / `MAG2` 日志 | 原始磁场强度，**未经 EKF** |
| **原始空速** | `AIRSPEED` MAVLink / `ARSP` 日志 | 空速计直接测量，**未经 EKF** |
| **电池电压/电流** | `SYS_STATUS` / `BATTERY_STATUS` | ADC 直接测量 |
| **测距仪距离** | `DISTANCE_SENSOR` | 直接测量 |

### 关键代码路径

```cpp
// 原始 GPS
AP_GPS::location(i)      // GPS_RAW_INT.lat/lng/alt
AP_GPS::ground_speed(i)  // GPS_RAW_INT.ground_speed
AP_GPS::ground_course(i) // GPS_RAW_INT.course

// 原始气压计
AP_Baro::get_altitude()  // BARO 日志 altitude
AP_Baro::get_pressure()  // BARO 日志 pressure

// 原始 IMU
AP_InertialSensor::get_gyro()   // IMU 日志 gyro_x/y/z
AP_InertialSensor::get_accel()  // IMU 日志 accel_x/y/z

// 原始空速
AP_Airspeed::get_airspeed()     // AIRSPEED 消息 / ARSP 日志
AP_Airspeed::get_raw_airspeed() // 未滤波原始值
```

---

## 三、部分融合 / 有条件融合的数据

这些数据根据条件可能使用传感器原始值，也可能使用融合后的估计值。

| 数据 | 来源 | 融合说明 |
|-----|------|---------|
| **VFR_HUD.airspeed** | 空速计 → 可选 AHRS 约束 | 默认直接传感器；Plane 可能用 AHRS 估计 |
| **AHRS 空速估计** | `airspeed_EAS()` | 传感器 + GPS 地速约束（`AHRS_WIND_MAX > 0` 时）|
| **合成空速** | EKF3 风估计 | 空速计失效时：`\|地速 - 风估计\|` |
| **TECS 高度变化率** | EKF 垂直速度 → 气压计互补滤波 | EKF 可用时用 EKF，否则用气压计 + 加速度计 |
| **定高控制器高度** | EKF → 气压计 | EKF 失效时回退到纯气压计 |

### AHRS 空速估计的 GPS 约束

```cpp
// libraries/AP_AHRS/AP_AHRS.cpp
if (_wind_max > 0 && AP::gps().status() >= AP_GPS_FixType::FIX_2D) {
    const float gnd_speed = AP::gps().ground_speed();
    float true_airspeed = airspeed_ret * get_EAS2TAS();
    // 将真空速限制在 [地速 - 最大风速, 地速 + 最大风速] 范围内
    true_airspeed = constrain_float(true_airspeed,
                                    gnd_speed - _wind_max,
                                    gnd_speed + _wind_max);
    airspeed_ret = true_airspeed / get_EAS2TAS();
}
```

**注意**：此约束仅在 `AHRS_WIND_MAX > 0` 时启用，默认通常为 0（不启用）。

---

## 四、MAVLink 消息数据来源对照表

| MAVLink 消息 | 字段 | 数据来源 | 是否融合 |
|-------------|------|---------|---------|
| **GLOBAL_POSITION_INT** | lat/lng | `ahrs.get_location()` | 融合 |
| **GLOBAL_POSITION_INT** | alt (AMSL) | `ahrs.get_location().alt` | 融合 |
| **GLOBAL_POSITION_INT** | relative_alt | `ahrs.get_relative_position_D_home()` | 融合 |
| **GLOBAL_POSITION_INT** | vx/vy/vz | `ahrs.get_velocity_NED()` | 融合 |
| **GLOBAL_POSITION_INT** | hdg | `ahrs.yaw_sensor` | 融合 |
| **ATTITUDE** | roll/pitch/yaw | `ahrs.get_roll/pitch/yaw_rad()` | 融合 |
| **ATTITUDE** | rollspeed/pitchspeed/yawspeed | `ahrs.get_gyro()` | 融合（修正漂移后）|
| **VFR_HUD** | airspeed | `AP_Airspeed::get_airspeed()` | **直接测量** |
| **VFR_HUD** | groundspeed | `ahrs.groundspeed()` | 融合 |
| **VFR_HUD** | heading | `ahrs.get_yaw_deg()` | 融合 |
| **VFR_HUD** | alt | `global_position_current_loc.alt` | 融合 |
| **VFR_HUD** | climb_rate | `ahrs.get_velocity_NED()` 或 `get_vert_pos_rate_D()` | 融合 |
| **GPS_RAW_INT** | lat/lng/alt | `AP_GPS::location()` | **直接测量** |
| **GPS_RAW_INT** | ground_speed | `AP_GPS::ground_speed()` | **直接测量** |
| **GPS_RAW_INT** | course | `AP_GPS::ground_course()` | **直接测量** |
| **GPS_RAW_INT** | satellites | `AP_GPS::num_sats()` | **直接测量** |
| **LOCAL_POSITION_NED** | x/y/z | `ahrs.get_relative_position_NED_origin_float()` | 融合 |
| **LOCAL_POSITION_NED** | vx/vy/vz | `ahrs.get_velocity_NED()` | 融合 |
| **WIND** | direction/speed/z | `ahrs.wind_estimate()` | 融合 |
| **AIRSPEED** | airspeed | `AP_Airspeed::get_airspeed()` | **直接测量** |
| **SCALED_IMU** / **RAW_IMU** | x/y/z | `AP_InertialSensor::get_gyro/accel()` | **直接测量** |
| **DISTANCE_SENSOR** | current_distance | 测距仪驱动 | **直接测量** |

---

## 五、日志记录数据来源对照表

| 日志类型 | 字段 | 数据来源 | 是否融合 |
|---------|------|---------|---------|
| **POS** | Lat/Lng/Alt | `ahrs.get_location()` | 融合 |
| **POS** | RelHomeAlt/RelOriginAlt | `ahrs.get_relative_position_D_home/origin()` | 融合 |
| **ATT** | Roll/Pitch/Yaw | `ahrs.roll/pitch/yaw_sensor` | 融合 |
| **ATT** | DesRoll/DesPitch/DesYaw | 导航目标 | 控制输出 |
| **CTUN** | roll/pitch | `ahrs.roll/pitch_sensor` | 融合 |
| **CTUN** | airspeed_estimate | `ahrs.airspeed_EAS()` | 部分融合 |
| **NTUN** | wp_distance/target_bearing | 导航控制器 | 控制输出 |
| **BARO** | altitude/pressure | `AP_Baro::get_altitude/pressure()` | **直接测量** |
| **IMU** | gyro/accel | `AP_InertialSensor::get_gyro/accel()` | **直接测量** |
| **ARSP** | airspeed/raw_airspeed | `AP_Airspeed::get_airspeed/raw_airspeed()` | **直接测量** |
| **GPS** | Lat/Lng/Alt/Spd | `AP_GPS::location/ground_speed()` | **直接测量** |
| **MAG** | MagX/Y/Z | `AP_Compass::get_field()` | **直接测量** |
| **WIND** | direction/speed | `ahrs.wind_estimate()` | 融合 |

---

## 六、控制回路数据来源速查

| 控制功能 | 使用数据 | 来源 | 是否融合 |
|---------|---------|------|---------|
| **姿态控制 (PID)** | roll/pitch/yaw | `ahrs.get_roll/pitch/yaw_rad()` | 融合 |
| **导航 (航点跟踪)** | 位置、速度 | `ahrs.get_location()` / `get_velocity_NED()` | 融合 |
| **定高/高度保持** | 高度、爬升率 | `ahrs.get_relative_position_D_home()` | 融合 |
| **TECS (能量管理)** | 高度 | `ahrs.get_relative_position_D_home()` | 融合 |
| **TECS (能量管理)** | 空速 | `ahrs.airspeed_EAS()` | 部分融合 |
| **TECS (能量管理)** | 加速度 | `ahrs.get_accel_ef()` + `AP::ins().get_accel()` | 混合 |
| **空速保护** | 空速 | `AP_Airspeed::get_airspeed()` → `ahrs.airspeed_EAS()` | 直接 → 部分融合 |
| **起飞检测** | 加速度、空速 | `AP::ins().get_accel()` / `AP_Airspeed::get_airspeed()` | **直接测量** |
| **罗盘校准** | 磁场强度 | `AP_Compass::get_field()` | **直接测量** |

---

## 七、AHRS 数据来源架构图

```
                    ┌─────────────────────────────────────────┐
                    │           AP_AHRS (前端)                 │
                    │  统一接口：get_location(), get_roll()    │
                    │         get_velocity_NED() 等            │
                    └─────────────────────────────────────────┘
                                      │
          ┌───────────────────────────┼───────────────────────────┐
          │                           │                           │
    ┌─────▼─────┐            ┌───────▼────────┐         ┌───────▼──────┐
    │  EKF3     │            │    EKF2        │         │    DCM       │
    │ (主要)    │            │  (备用)        │         │ (最终回退)   │
    └─────┬─────┘            └───────┬────────┘         └───────┬──────┘
          │                          │                          │
    ┌─────▼──────────────────────────▼──────────────────────────▼─────┐
    │                        传感器输入层                              │
    │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────┐ │
    │  │  GPS    │  │  IMU    │  │  Baro   │  │ Compass │  │Airspeed│ │
    │  │(原始)   │  │(原始)   │  │(原始)   │  │(原始)   │  │(原始)  │ │
    │  └─────────┘  └─────────┘  └─────────┘  └─────────┘  └──────┘ │
    └─────────────────────────────────────────────────────────────────┘
```

---

## 八、一句话记忆法则

> **通过 `ahrs.xxx()` 获取的 = EKF 融合后的**
>
> **通过 `AP_传感器.xxx()` 获取的 = 原始传感器直接测量**
>
> **MAVLink 中 `RAW` / `SCALED_IMU` / `GPS_RAW` = 原始数据**
>
> **MAVLink 中 `GLOBAL` / `ATTITUDE` / `VFR_HUD` (除 airspeed) = 融合数据**

### 快速判断方法

| 你想看... | 应该看哪个 MAVLink 消息 |
|----------|------------------------|
| EKF 估计的位置/姿态 | `GLOBAL_POSITION_INT`, `ATTITUDE` |
| 原始 GPS 接收机输出 | `GPS_RAW_INT` |
| 原始空速计读数 | `AIRSPEED` |
| 原始 IMU 数据 | `SCALED_IMU` / `RAW_IMU` |
| EKF 估计的风 | `WIND` |
| 原始气压计数据 | `SCALED_PRESSURE` |

---

## 九、关键文件路径汇总

| 功能 | 文件路径 |
|------|----------|
| AHRS 统一接口 | `libraries/AP_AHRS/AP_AHRS.h` / `.cpp` |
| EKF3 核心融合 | `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` |
| EKF3 测量输入 | `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` |
| EKF3 输出接口 | `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp` |
| GPS 驱动 | `libraries/AP_GPS/AP_GPS.cpp` |
| 气压计驱动 | `libraries/AP_Baro/AP_Baro.cpp` |
| IMU 驱动 | `libraries/AP_InertialSensor/AP_InertialSensor.cpp` |
| 空速计驱动 | `libraries/AP_Airspeed/AP_Airspeed.cpp` |
| 磁力计驱动 | `libraries/AP_Compass/AP_Compass.cpp` |
| MAVLink 消息发送 | `libraries/GCS_MAVLink/GCS_Common.cpp` |
| 日志记录 (AHRS) | `libraries/AP_AHRS/AP_AHRS_Logging.cpp` |
| 日志记录 (空速) | `libraries/AP_Airspeed/AP_Airspeed.cpp` |
| 日志记录 (GPS) | `libraries/AP_GPS/AP_GPS.cpp` |
| 日志记录 (气压计) | `libraries/AP_Baro/AP_Baro_Logging.cpp` |
| 日志记录 (IMU) | `libraries/AP_InertialSensor/AP_InertialSensor_Logging.cpp` |
| TECS 控制 | `libraries/AP_TECS/AP_TECS.cpp` |

---

*文档生成时间：2026-05-15*
*基于 ArduPilot 主分支代码分析*
