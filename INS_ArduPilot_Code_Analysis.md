# 惯导（INS）在 ArduPilot 中的代码解析

## 1. 整体架构

ArduPilot 的惯导系统分为三层：

```
┌─────────────────────────────────────────┐
│  AP_InertialNav (高层封装)              │  ← 提供位置/速度接口
├─────────────────────────────────────────┤
│  AP_NavEKF3 (EKF 状态估计)              │  ← 融合惯导 + GPS/气压计/磁力计
├─────────────────────────────────────────┤
│  AP_InertialSensor (底层传感器驱动)      │  ← 读取 IMU 原始数据
└─────────────────────────────────────────┘
```

---

## 2. 底层：AP_InertialSensor — 传感器数据采集

**文件**: `libraries/AP_InertialSensor/AP_InertialSensor.h`

这是 IMU 硬件抽象层，核心职责：

### 2.1 提供的数据

```cpp
// AP_InertialSensor.h:110-135
const Vector3f &get_gyro(uint8_t i) const { return _gyro[i]; }      // 角速度 (rad/s)
const Vector3f &get_accel(uint8_t i) const { return _accel[i]; }    // 加速度 (m/s²)

// 更关键的：delta angle / delta velocity（积分后的增量）
bool get_delta_angle(uint8_t i, Vector3f &delta_angle, float &delta_angle_dt) const;
bool get_delta_velocity(uint8_t i, Vector3f &delta_velocity, float &delta_velocity_dt) const;
```

**为什么用 delta angle/velocity 而不是 raw gyro/accel？**

- **delta angle** = 陀螺仪在时间段内的角度积分（消除圆锥误差）
- **delta velocity** = 加速度计在时间段内的速度增量（消除划桨误差）
- 这是高精度惯导的标准做法，避免在快速旋转时产生积分误差

### 2.2 update() 主循环

```cpp
// AP_InertialSensor.cpp:1897
void AP_InertialSensor::update(void)
{
    wait_for_sample();  // 等待传感器新数据

    for (uint8_t i=0; i<_backend_count; i++) {
        _backends[i]->update();  // 调用各后端驱动更新
    }

    // 健康检查、主传感器选择...
}
```

### 2.3 get_delta_angle 实现

```cpp
// AP_InertialSensor.cpp:2156
bool AP_InertialSensor::get_delta_angle(uint8_t i, Vector3f &delta_angle, float &delta_angle_dt) const
{
    if (_delta_angle_valid[i]) {
        delta_angle = _delta_angle[i];  // 返回已积分的 delta angle
        return true;
    } else if (get_gyro_health(i)) {
        // 降级：用原始 gyro * dt 近似
        delta_angle = get_gyro(i) * get_delta_time();
        return true;
    }
    return false;
}
```

---

## 3. 中层：AP_NavEKF3 — 惯导 + 多传感器融合

**文件**: `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`

这是核心状态估计器，24 状态 EKF。

### 3.1 主循环 UpdateFilter

```cpp
// AP_NavEKF3_core.cpp:625
void NavEKF3_core::UpdateFilter(bool predict)
{
    // 1. 读取 IMU 数据
    readIMUData(predict);

    if (runUpdates) {
        // 2. 惯导机械编排（ strapdown 方程）
        UpdateStrapdownEquationsNED();

        // 3. 协方差预测
        CovariancePrediction(nullptr);

        // 4. 各种传感器融合校正
        SelectMagFusion();      // 磁力计
        SelectVelPosFusion();   // GPS
        SelectFlowFusion();     // 光流
        SelectTasFusion();      // 空速
        // ...
    }

    // 5. 输出状态外推到当前时间
    calcOutputStates();
}
```

### 3.2 读取 IMU 数据 readIMUData

```cpp
// AP_NavEKF3_Measurements.cpp:395
void NavEKF3_core::readIMUData(bool startPredictEnabled)
{
    // 从 AP_InertialSensor 读取 delta velocity
    readDeltaVelocity(accel_index_active, imuDataNew.delVel, imuDataNew.delVelDT);

    // 从 AP_InertialSensor 读取 delta angle
    readDeltaAngle(gyro_index_active, imuDataNew.delAng, imuDataNew.delAngDT);

    // 降采样到 EKF 目标频率（约 100Hz），使用四元数累加避免圆锥误差
    imuQuatDownSampleNew.rotate(imuDataNew.delAng);
    imuDataDownSampledNew.delVel += deltaRotMat * imuDataNew.delVel;

    // 存入 FIFO 缓冲区（EKF 使用延迟时间地平线）
    if (满足时间阈值) {
        storedIMU.push_youngest_element(imuDataDownSampledNew);
    }
}
```

### 3.3 机械编排方程 UpdateStrapdownEquationsNED

这是**纯惯导的核心**——捷联惯导算法：

```cpp
// AP_NavEKF3_core.cpp:743
void NavEKF3_core::UpdateStrapdownEquationsNED()
{
    // ========== 姿态更新 ==========
    // 用 delta angle 旋转四元数，并补偿地球自转
    stateStruct.quat.rotate(delAngCorrected - prevTnb * earthRateNED * imuDataDelayed.delAngDT);
    stateStruct.quat.normalize();

    // ========== 速度更新 ==========
    // 将机体坐标系的 delta velocity 转换到导航坐标系（NED）
    Vector3F delVelNav = prevTnb.mul_transpose(delVelCorrected);
    delVelNav.z += GRAVITY_MSS * imuDataDelayed.delVelDT;  // 补偿重力

    // 积分得到速度
    stateStruct.velocity += delVelNav;

    // ========== 位置更新 ==========
    // 梯形积分：position += (v_new + v_old) / 2 * dt
    Vector3F lastVelocity = stateStruct.velocity;
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT * 0.5f);
}
```

**这就是惯导的本质**：
- 陀螺仪 → 积分 → 姿态
- 加速度计（去重力后）→ 积分 → 速度 → 再积分 → 位置

### 3.4 为什么需要 EKF 融合？

纯惯导的误差会随时间**指数发散**：
- 陀螺零偏 → 姿态误差 → 重力分解错误 → 速度误差 → 位置误差（t³ 增长）
- 加速度计零偏 → 速度误差 → 位置误差（t² 增长）

所以 EKF 用以下观测来**校正**惯导漂移：

| 传感器 | 校正的状态 | 代码位置 |
|--------|-----------|---------|
| GPS | 位置、速度 | `SelectVelPosFusion()` |
| 气压计 | 高度 | `SelectVelPosFusion()` |
| 磁力计 | 航向 | `SelectMagFusion()` |
| 光流 | 水平速度 | `SelectFlowFusion()` |
| 空速管 | 风速 | `SelectTasFusion()` |

### 3.5 输出状态外推 calcOutputStates

```cpp
// AP_NavEKF3_core.cpp:819
void NavEKF3_core::calcOutputStates()
{
    // EKF 在"延迟时间地平线"上运行（处理历史数据）
    // 这里把结果外推到"当前时间"，供控制环使用

    // 同样做捷联积分，但用最新的 IMU 数据
    outputDataNew.quat *= deltaQuat;
    outputDataNew.velocity += delVelNav;
    outputDataNew.position += ...;

    // 垂直通道用互补滤波（三阶，参考 Widnall & Sinha 论文）
    // 融合 EKF 高度和惯导高度变化率
}
```

---

## 4. 高层：AP_InertialNav — 封装接口

**文件**: `libraries/AP_InertialNav/AP_InertialNav.h`

这个库现在只是一个**薄封装**，实际数据全部来自 EKF：

```cpp
// AP_InertialNav.h:12-16
/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
*/

// AP_InertialNav.cpp:14
void AP_InertialNav::update(bool high_vibes)
{
    // 从 EKF 获取 NE 位置
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE_origin_float(posNE)) {
        _relpos_cm.x = posNE.x * 100;  // m → cm
        _relpos_cm.y = posNE.y * 100;
    }

    // 从 EKF 获取 D 位置（高度）
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin_float(posD)) {
        _relpos_cm.z = -posD * 100;  // NED → NEU
    }

    // 从 EKF 获取 NED 速度
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100;
    }
}
```

> **注意**：AP_InertialNav 本身**不再做惯导积分**，它只是 EKF 结果的搬运工。这是现代飞控的设计趋势——所有导航状态统一由 EKF 估计。

---

## 5. 完整数据流图

```
IMU 硬件 (MPU6000, BMI088, etc.)
    │
    ▼
┌─────────────────────────────┐
│ AP_InertialSensor_Backend   │  ← 各芯片驱动 (SPI/I2C 读取)
│ (e.g. AP_InertialSensor_Invensense)
└─────────────────────────────┘
    │
    ▼
┌─────────────────────────────┐
│ AP_InertialSensor::update() │  ← 滤波、校准、健康检查
│  - _publish_gyro()          │
│  - _publish_accel()         │
│  - 计算 delta_angle         │
│  - 计算 delta_velocity      │
└─────────────────────────────┘
    │
    ▼
┌─────────────────────────────┐
│ NavEKF3_core::readIMUData() │  ← 读取 delta_angle/delta_velocity
│  - 降采样到 100Hz            │
│  - 四元数累加防圆锥误差       │
│  - 存入 FIFO 缓冲区          │
└─────────────────────────────┘
    │
    ▼
┌─────────────────────────────┐
│ UpdateStrapdownEquationsNED │  ← 捷联惯导机械编排
│  - 姿态：四元数 + deltaAngle │
│  - 速度：积分 deltaVelocity  │
│  - 位置：梯形积分            │
└─────────────────────────────┘
    │
    ▼
┌─────────────────────────────┐
│ EKF 融合校正                 │
│  - GPS → 位置/速度           │
│  - 气压计 → 高度             │
│  - 磁力计 → 航向             │
│  - ...                      │
└─────────────────────────────┘
    │
    ▼
┌─────────────────────────────┐
│ calcOutputStates()          │  ← 外推到当前时间
└─────────────────────────────┘
    │
    ▼
┌─────────────────────────────┐
│ AP_InertialNav::update()    │  ← 封装为 cm/cm/s 单位
│  - get_position_neu_cm()    │
│  - get_velocity_neu_cms()   │
└─────────────────────────────┘
    │
    ▼
  飞行控制律 (PosHold, Auto, 等)
```

---

## 6. 关键概念总结

| 概念 | 代码体现 | 作用 |
|------|---------|------|
| **捷联惯导 (Strapdown INS)** | `UpdateStrapdownEquationsNED()` | 用 IMU 直接积分得到 PVA |
| **Delta Angle** | `get_delta_angle()` | 陀螺仪角度增量（防圆锥误差） |
| **Delta Velocity** | `get_delta_velocity()` | 加速度计速度增量（防划桨误差） |
| **EKF 融合** | `SelectVelPosFusion()` 等 | 用外部观测校正惯导漂移 |
| **延迟时间地平线** | `storedIMU` FIFO 缓冲区 | 处理传感器延迟不一致 |
| **输出外推** | `calcOutputStates()` | 把延迟估计结果推到当前时刻 |

---

## 7. 核心要点

1. **纯惯导**由 `UpdateStrapdownEquationsNED()` 实现，通过积分 IMU 数据得到姿态、速度、位置
2. **误差发散**由 EKF 通过融合 GPS、气压计、磁力计等外部观测来抑制
3. **AP_InertialNav** 已不再做独立惯导计算，只是 EKF 结果的封装层
