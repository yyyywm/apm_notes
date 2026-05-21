# IMU 位置偏移补偿详解

## 一、IMU 偏移量是相对于什么的？

**相对于机体坐标系原点（body frame origin）**

机体坐标系原点的定义：
- **X 轴**：机头方向为正
- **Y 轴**：右侧为正
- **Z 轴**：向下为正（NED 坐标系）
- **原点**：通常定义为**飞行器重心（Center of Gravity）**

---

## 二、为什么要设置这个偏移量？

### 场景 1：IMU 在重心（理想情况）

```
        机头
          ↑
    [IMU] ● ← 重心 = 机体原点
          |
        机尾

INS_POS1_X = 0
INS_POS1_Y = 0
INS_POS1_Z = 0

→ 不需要补偿，完美
```

### 场景 2：IMU 不在重心（实际情况）

```
        机头
          ↑
          |
    [IMU] ●────┐  ← IMU 在机头前方 0.1m
          |    |
          ●    │  ← 重心（机体原点）
          |    |
        机尾   │
               └─ 距离 = 0.1m

INS_POS1_X = 0.1  (IMU 在原点前方 0.1m)
INS_POS1_Y = 0
INS_POS1_Z = 0
```

---

## 三、不设置会发生什么？

### 飞机在俯仰（抬头/低头）

```
抬头时（绕 Y 轴旋转）：

    抬头前：        抬头后：

    →→→→           ↗
    [IMU]          [IMU]  ← 画了一个圆弧
    ●────┐         ●──┐
         │            │
         ●            ●

    IMU 位置速度 = 角速度 × 距离
    v = ω × r = 2 rad/s × 0.1m = 0.2 m/s

    但飞机重心其实没动（或动得很少）
```

**EKF 内部计算在 IMU 位置进行**，如果 IMU 在机头：
- 飞机抬头时，IMU 画了一个圆弧，有向心加速度
- EKF 以为飞机在前后运动
- **实际飞机重心没怎么动**

**输出补偿**：把 IMU 位置的估计，转换到重心位置

```
重心速度 = IMU 速度 - (角速度 × IMU偏移)
```

---

## 四、代码中的具体计算

### 4.1 补偿计算

文件：`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:879-894`

```cpp
// 角速度（陀螺仪读数）
Vector3F angRate = dal.ins().get_gyro(gyro_index_active).toftype();

// 杠杆速度：v = ω × r
// IMU 相对重心的速度（机体坐标系）
Vector3F velBodyRelIMU = angRate % (- accelPosOffset);

// 转换到地球坐标系
velOffsetNED = Tbn_temp * velBodyRelIMU;

// IMU 相对重心的位置（地球坐标系）
posOffsetNED = Tbn_temp * (- accelPosOffset);
```

### 4.2 输出时应用补偿

文件：`libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp`

```cpp
// 输出的是重心的速度，不是 IMU 的速度
vel = (outputDataNew.velocity + velOffsetNED).tofloat();

// 输出的是重心的位置
posD = outputDataNew.position.z + posOffsetNED.z;
```

---

## 五、实际例子

假设：
- IMU 安装在机头前方 10cm：`INS_POS1_X = 0.1`
- 飞机在做快速俯仰：角速度 ω = 2 rad/s

### 不设置参数时

- EKF 输出：飞机在前后震荡，速度 ±0.2 m/s
- 实际：飞机重心几乎没动
- 控制回路：拼命修正，导致震荡

### 设置参数后

- EKF 计算 IMU 速度 = 0.2 m/s
- 补偿：`velOffsetNED = ω × (-0.1) = -0.2 m/s`
- 输出重心速度 = 0.2 + (-0.2) = **0**
- 控制回路：知道重心没动，不瞎修正

---

## 六、EKF 为什么不自己估计这个偏移？

### 不可观测性（Observability）问题

IMU 位置偏移和某些导航状态是**耦合的**，在没有特定机动的情况下无法区分：

| 现象 | 可能原因 A | 可能原因 B |
|------|-----------|-----------|
| 水平速度有偏移 | IMU 安装位置有 X 偏移 | 加速度计零偏 |
| 高度估计漂移 | IMU 安装位置有 Z 偏移 | 气压计误差 |

EKF 的 24 个状态已经包含了陀螺零偏、加速度计零偏、风速等。如果再增加 3 个位置偏移状态，**协方差矩阵会出现秩亏**，导致估计不稳定。

### 对比：哪些偏移 EKF 会自己估计？

| 偏移类型 | 是否 EKF 估计 | 原因 |
|---------|-------------|------|
| **陀螺零偏** (`gyro_bias`) | ✅ 是 | 可观测（静止时重力方向不变，旋转漂移可检测） |
| **加速度计零偏** (`accel_bias`) | ✅ 是 | 可观测（多方向机动可分离） |
| **地磁场** (`earth_magfield`) | ✅ 是 | 可观测（旋转时磁场方向变化） |
| **机体磁场** (`body_magfield`) | ✅ 是 | 可观测（旋转时软磁干扰模式变化） |
| **风速** (`wind_vel`) | ✅ 是 | 可观测（空速与地速差异） |
| **IMU 安装位置** (`accelPosOffset`) | ❌ 否 | 不可观测（与零偏耦合） |

---

## 七、参数配置

### 参数名称

- `INS_POS1_X` / `INS_POS1_Y` / `INS_POS1_Z`：IMU1 的位置偏移
- `INS_POS2_X` / `INS_POS2_Y` / `INS_POS2_Z`：IMU2 的位置偏移
- `INS_POS3_X` / `INS_POS3_Y` / `INS_POS3_Z`：IMU3 的位置偏移

### 参数定义

文件：`libraries/AP_InertialSensor/AP_InertialSensor_Params.cpp:68-91`

```cpp
// @Param: POS_X
// @DisplayName: IMU accelerometer X position
// @Description: X position of the first IMU Accelerometer in body frame.
//   Positive X is forward of the origin.
//   Attention: The IMU should be located as close to the vehicle c.g.
//   as practical so that the value of this parameter is minimised.
//   Failure to do so can result in noisy navigation velocity measurements
//   due to vibration and IMU gyro noise.
// @Units: m
// @Range: -5 5
```

### 默认值

- 默认全为 **0**（假设 IMU 在机体原点）
- 如果实际有偏移但不设置，**补偿为 0**，产生误差

---

## 八、总结

| 问题 | 答案 |
|------|------|
| **偏移量是什么** | IMU 到飞行器重心的距离 |
| **为什么要设置** | EKF 在 IMU 位置计算，但飞控需要重心的状态 |
| **不设置会怎样** | 飞机旋转时，EKF 以为重心在动，控制回路会瞎修正 |
| **怎么测量** | 用尺子量 IMU 到重心的 XYZ 距离，填入参数 |
| **EKF 能自己估计吗** | **不能**，不可观测，与零偏耦合 |
| **精度要求** | 尽量让 IMU 靠近重心，减小参数值 |

**关键**：这个偏移是**物理安装位置**，不是传感器误差。EKF 无法从数据中区分"IMU 偏移造成的加速度"和"飞机真的在加速"，所以必须手动配置。
