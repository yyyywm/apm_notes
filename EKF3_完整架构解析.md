# EKF3 (NavEKF3) 完整架构解析

## 一、EKF3 是什么？

**EKF3 = Extended Kalman Filter 3**，是 ArduPilot 的扩展卡尔曼滤波器，用于**多传感器融合状态估计**。

它基于 PX4/ecl 的推导，核心任务是：**把多个传感器的不完整、有噪声的测量，融合成对飞行器状态的"最优估计"**。

---

## 二、EKF3 估计哪些状态？

在 `libraries/AP_NavEKF3/AP_NavEKF3_core.h` 中定义了 **24 个状态**：

| 索引 | 状态 | 含义 |
|---|---|---|
| 0-3 | `quat` | 四元数（姿态）|
| 4-6 | `velocity` | NED 坐标系速度（m/s）|
| 7-9 | `position` | NED 坐标系位置（m）|
| 10-12 | `gyro_bias` | 陀螺仪零偏（rad）|
| 13-15 | `accel_bias` | 加速度计零偏（m/s）|
| 16-18 | `earth_magfield` | 地磁场（地球坐标系）|
| 19-21 | `body_magfield` | 机体磁场（软磁干扰）|
| 22-23 | `wind_vel` | 水平风速（N/E）|

这些状态构成 `stateStruct`，是 EKF 的**核心内部状态**。

---

## 三、数据来源有哪些？

EKF3 从多种传感器获取数据，每种数据通过**环形缓冲区**处理以解决时间延迟问题：

| 传感器 | 数据类型 | 缓冲区 | 用途 |
|---|---|---|---|
| **IMU** | 角增量 + 速度增量 | `storedIMU` | 预测步骤（核心时钟）|
| **GPS** | 经纬高 + NED 速度 + 航向 | `storedGPS` | 绝对位置/速度/高度 |
| **磁力计** | 机体磁场三轴 | `storedMag` | 航向估计 |
| **气压计** | 气压高度 | `storedBaro` | 高度参考 |
| **测距仪** | 对地距离 | `storedRange` | 相对高度/地形估计 |
| **空速管** | 真空速 | `storedTAS` | 风速估计 |
| **光流** | 角速度 + 机体角速度 | `storedOF` | 相对速度/地形估计 |
| **外部导航** | 位置/速度/姿态 | `storedExtNav` | VIO/动捕数据 |
| **信标** | 到已知位置信标的距离 | `rngBcn` | 室内定位 |
| **轮速编码器** | 轮转角 | `storedWheelOdm` | 地面车辆 |
| **机体里程计** | 机体坐标系速度 | `storedBodyOdm` | 视觉里程计 |

---

## 四、融合流程：数据怎么变成状态估计？

主循环在 `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:626-725` 的 `UpdateFilter()` 中：

```
UpdateFilter()
├── controlFilterModes()          // 确定当前 aiding 模式
├── readIMUData()                 // 读取 IMU 数据到缓冲区
│
├── 如果到了融合时刻 (runUpdates):
│   ├── UpdateStrapdownEquationsNED()   // 用 IMU 预测状态
│   ├── CovariancePrediction()          // 预测协方差增长
│   ├── runYawEstimatorPrediction()     // GSF 航向估计器预测
│   ├── SelectMagFusion()               // 融合磁力计/航向
│   ├── SelectVelPosFusion()            // 融合 GPS/气压计/外部导航
│   ├── runYawEstimatorCorrection()     // GSF 校正
│   ├── SelectRngBcnFusion()            // 融合信标
│   ├── SelectFlowFusion()              // 融合光流
│   ├── SelectBodyOdomFusion()          // 融合机体里程计
│   ├── SelectTasFusion()               // 融合空速
│   ├── SelectBetaDragFusion()          // 融合侧滑/阻力
│   └── updateFilterStatus()            // 更新状态标志
│
└── calcOutputStates()              // 输出预测器（关键！）
```

### 4.1 预测步骤（Prediction）

`UpdateStrapdownEquationsNED()` 用 IMU 数据做**惯性导航推算**：
- 四元数积分 -> 得到新姿态
- 加速度旋转到 NED + 重力 -> 积分得速度
- 速度积分 -> 得到位置

`CovariancePrediction()` 让**协方差矩阵 P** 增长（不确定性随时间增大）。

### 4.2 融合步骤（Fusion）

每个融合函数都做同样的事：

```
1. 计算预测值（用当前状态预测传感器读数）
2. 计算创新（innovation）= 预测值 - 实际测量值
3. 计算创新方差 = H * P * H^T + R
4. 一致性检查：innovation^2 / variance < 阈值
5. 如果通过检查：
   K = P * H^T / innovation_variance   // 卡尔曼增益
   state = state + K * innovation       // 修正状态
   P = P - K * H * P                    // 修正协方差
```

---

## 五、关键概念：状态 vs 输出

这是 EKF3 最容易混淆的地方。

### `stateStruct` — 融合时域的状态

- 存在于**延迟时域**（delayed time horizon，通常延迟 100-250ms）
- 这是 EKF 数学运算的地方
- 数据对齐了所有传感器的延迟
- 跟踪的是 **IMU 位置**，不是机体中心

### `outputDataNew` — 输出时域的状态

- 存在于**当前时域**（current time，实时）
- 由 `calcOutputStates()` 生成
- 跟踪的是 **机体中心**（有 `posOffsetNED` 修正）
- 给飞控用的就是这个

### 为什么要分离？

```
传感器有延迟 -> EKF 必须在延迟时域做融合 -> 但飞控需要实时数据
                    |
            calcOutputStates()
            用互补滤波从延迟时域"推"到当前时域
                    |
            输出平滑的实时状态给飞控
```

`calcOutputStates()` 本质上是一个**并行的惯性导航系统**，不断被 EKF 状态"拉"向正确值，但输出是平滑连续的，不会有 EKF 融合时的跳变。

---

## 六、三种 Aiding 模式

在 `libraries/AP_NavEKF3/AP_NavEKF3_core.h` 中定义：

| 模式 | 含义 | 场景 |
|---|---|---|
| `AID_ABSOLUTE` | 绝对位置 aiding | GPS、外部导航、信标可用 |
| `AID_RELATIVE` | 相对位置 aiding | 光流、轮速编码器、机体里程计 |
| `AID_NONE` | 无位置 aiding | 只有姿态和高度 |

### 模式切换逻辑

```
启动时 -> AID_NONE（只有 IMU + 气压计）

如果有 GPS/外部导航/信标且通过检查 -> AID_ABSOLUTE
如果有光流/里程计且 GPS 不可用 -> AID_RELATIVE

如果所有 aiding 源超时：
    AID_ABSOLUTE -> AID_NONE
    AID_RELATIVE -> AID_NONE
```

### 各模式下能估计什么？

| 状态 | AID_NONE | AID_RELATIVE | AID_ABSOLUTE |
|---|---|---|---|
| 姿态 | ✓ | ✓ | ✓ |
| 高度 | ✓ | ✓ | ✓ |
| 垂直速度 | ✓（受限） | ✓ | ✓ |
| 水平速度 | ✗（地面合成零速） | ✓ | ✓ |
| 水平位置 | ✗（常值约束） | ✓（会漂移） | ✓（全局参考）|

---

## 七、EKF3 输出什么？

通过 `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp` 提供：

| 输出 | 函数 | 来源 |
|---|---|---|
| 姿态（四元数/欧拉角） | `getQuaternion()` / `getEulerAngles()` | `outputDataNew.quat` |
| NED 速度 | `getVelNED()` | `outputDataNew.velocity + velOffsetNED` |
| NED 位置 | `getPosNE()` / `getPosD()` | `outputDataNew.position + posOffsetNED` |
| 经纬高 | `getLLH()` | 原点 + NED 位置转换 |
| 陀螺零偏 | `getGyroBias()` | `stateStruct.gyro_bias` |
| 加计零偏 | `getAccelBias()` | `stateStruct.accel_bias` |
| 风速 | `getWind()` | `stateStruct.wind_vel` |
| 地磁场 | `getMagNED()` | `stateStruct.earth_magfield` |
| 相对地面高度 | `getHAGL()` | `terrainState - position.z` |
| 垂直速度（滤波后） | `getPosDownDerivative()` | `vertCompFiltState.vel` |
| 滤波器状态 | `getFilterStatus()` | `filterStatus` 标志位 |

---

## 八、"导航 EKF" vs "非导航 EKF"

`libraries/AP_NavEKF/AP_NavEKF_Source.h` 中的 `SourceXY`、`SourceZ`、`SourceYaw` 是**导航数据源**的分类。这里要区分几个概念：

### 8.1 什么是"导航"？

在飞控语境中，**导航（Navigation）** = **知道自己在哪、往哪去**。

- **导航状态**：位置、速度、航向
- **非导航状态**：姿态、传感器零偏、磁场、风速

### 8.2 EKF3 同时估计导航和非导航状态

EKF3 的 24 个状态中：
- **导航相关**：position（7-9）、velocity（4-6）、quat（0-3，姿态决定航向）
- **非导航相关**：gyro_bias、accel_bias、earth_magfield、body_magfield、wind_vel

### 8.3 "导航 EKF 数据源"的含义

`AP_NavEKF_Source` 定义的是**哪些传感器可以为导航提供参考**：

- `SourceXY`：谁能提供**水平位置**？-> GPS、信标、光流、外部导航、轮速编码器
- `SourceZ`：谁能提供**高度**？-> 气压计、测距仪、GPS、信标、外部导航
- `SourceYaw`：谁能提供**航向**？-> 磁力计、GPS、外部导航、GSF

**IMU 不在 Source 中**，因为 IMU 是**基础传感器**，不是"aiding 源"。IMU 数据始终进入 EKF，用于预测步骤。

### 8.4 为什么叫"NavEKF"？

- **Nav** = Navigation
- 强调这个 EKF 的主要目的是**导航状态估计**
- 与之对比，ArduPilot 还有：
  - **DCM**（Direction Cosine Matrix）：简单的姿态估计算法，无位置估计
  - **AHRS**（Attitude Heading Reference System）：更高层的抽象，可以选择后端（DCM、EKF2、EKF3）

### 8.5 AHRS 与 EKF3 的关系

```
飞行器代码
    |
AP_AHRS（姿态航向参考系统）
    | 选择后端
├── AP_AHRS_DCM（简单，无位置）
├── AP_AHRS_NavEKF2（旧版 EKF）
└── AP_AHRS_NavEKF3（当前主流）
        |
    NavEKF3（前端）
        | 管理多个核心
    NavEKF3_core × N（每个 IMU 一个）
```

所以 **AHRS 是接口层**，**EKF3 是实现层**。飞行模式代码调用 `ahrs.get_position()`，底层可能是 EKF3 在提供数据。

---

## 九、Frontend / Backend 架构

### NavEKF3（Frontend）

- 管理多个 `NavEKF3_core` 实例（每个 IMU 一个，最多 9 个）
- 根据健康度和误差评分选择**主核心（primary core）**
- 提供统一的公共 API
- 支持**lane switching**：当主核心不健康时，切换到备用核心

### NavEKF3_core（Backend）

- 每个核心绑定一个 IMU
- 独立运行完整的 EKF 算法
- 可以使用不同的传感器组合（GPS affinity、compass affinity）
- 多个核心并行运行，增加容错性

---

## 十、高度数据源详解

### SourceZ 枚举（`AP_NavEKF_Source.h:28-37`）

| 源 | 值 | 类型 | 说明 |
|---|---|---|---|
| `NONE` | 0 | 无 | 无高度 aiding，EKF 以 14Hz 融合常值 0 |
| `BARO` | 1 | 气压计 | 气压高度，通用后备 |
| `RANGEFINDER` | 2 | 测距仪 | 激光/超声波测距，低空精确 |
| `GPS` | 3 | GPS | GNSS 椭球高/海拔高 |
| `BEACON` | 4 | 信标 | 室内定位系统的高度 |
| `EXTNAV` | 6 | 外部导航 | VIO 等提供的 Z 轴位置 |

### 高度融合逻辑（`selectHeightForFusion()`）

```
1. 外部导航（EXTNAV）-> hgtMea = -extNavDataDelayed.pos.z
2. 测距仪（RANGEFINDER）-> hgtMea = range * cos(tilt) - terrainState
3. GPS -> hgtMea = gpsDataDelayed.hgt
4. 气压计（BARO）-> hgtMea = baroDataDelayed.hgt - baroHgtOffset
5. NONE -> hgtMea = 0.0f（常值约束）
```

### 测距仪高度融合的特殊处理

测距仪测的是**相对地面高度**，但 EKF 需要**绝对高度**。引入 `terrainState`（地形高度状态）：

```
测距仪原始读数 * cos(倾斜角) = 斜距修正为垂直距
hgtMea = 垂直距 - terrainState  →  得到相对于 EKF 原点的"绝对"高度
```

同时通过测距仪数据更新 `terrainState`：
```
terrainState = 测距仪读数 + 当前飞行器高度
```

### 相对高度（HAGL）计算

```
HAGL = terrainState - outputDataNew.position.z - posOffsetNED.z
```

即：**相对高度 = 地面高度 - 飞行器高度**

如果没有测距仪：
- `terrainState` 不会被有效更新
- `gndOffsetValid` 变为 false
- HAGL 不可靠，只能知道相对于 EKF 原点的高度变化

---

## 十一、完整数据流总结

```
┌─────────────────────────────────────────────────────────────┐
│                        传感器层                               │
│  IMU    GPS    磁力计   气压计   测距仪   光流   空速   外部导航  │
└─────────────────────────────────────────────────────────────┘
                           |
┌─────────────────────────────────────────────────────────────┐
│                      测量读取层                               │
│  readIMUData()  readGpsData()  readMagData()  ...           │
│  -> 存入环形缓冲区（解决传感器延迟差异）                          │
└─────────────────────────────────────────────────────────────┘
                           |
┌─────────────────────────────────────────────────────────────┐
│                      EKF 融合层（延迟时域）                     │
│  UpdateStrapdownEquationsNED()  <- IMU 预测                    │
│  SelectMagFusion()              <- 磁力计修正姿态               │
│  SelectVelPosFusion()           <- GPS/气压计修正位置速度高度     │
│  SelectFlowFusion()             <- 光流修正相对速度             │
│  SelectTasFusion()              <- 空速修正风速                │
│  ...                                                          │
│  -> stateStruct（24 状态）+ P（24×24 协方差）                   │
└─────────────────────────────────────────────────────────────┘
                           |
┌─────────────────────────────────────────────────────────────┐
│                    输出预测层（当前时域）                        │
│  calcOutputStates()                                           │
│  -> 互补滤波 + PI 跟踪，从延迟时域推到当前时域                    │
│  -> outputDataNew（实时平滑输出）                               │
└─────────────────────────────────────────────────────────────┘
                           |
┌─────────────────────────────────────────────────────────────┐
│                      输出接口层                                │
│  getPosNE()  getVelNED()  getEulerAngles()  getHAGL() ...    │
│  -> 飞行控制律、导航、地面站遥测                                │
└─────────────────────────────────────────────────────────────┘
```

---

## 十二、关键文件索引

| 文件 | 用途 |
|---|---|
| `libraries/AP_NavEKF3/AP_NavEKF3.h` | Frontend 类定义和公共 API |
| `libraries/AP_NavEKF3/AP_NavEKF3.cpp` | Frontend 实现（核心管理、lane 切换）|
| `libraries/AP_NavEKF3/AP_NavEKF3_core.h` | Backend 类定义、状态结构、数据缓冲区 |
| `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` | 主滤波器循环、预测、输出观测器、初始化 |
| `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp` | 模式控制、aiding 模式、风/磁学习、滤波器状态 |
| `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` | 传感器数据读取（GPS、气压计、磁、空速、测距仪）|
| `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp` | GPS、外部导航、气压计、测距仪融合 |
| `libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp` | 磁力计和航向融合 |
| `libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp` | 空速、侧滑、阻力融合 |
| `libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp` | 光流融合 |
| `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp` | 输出访问函数 |
| `libraries/AP_NavEKF/AP_NavEKF_Source.h` | 导航数据源枚举定义 |

---

## 十三、AP_AHRS_NavEKF3 vs NavEKF3 的区别

这是最容易混淆的两个类，它们处于**完全不同的层次**：

### 架构层次图

```
┌─────────────────────────────────────────┐
│           飞行模式/控制代码                │
│    plane.nav_controller->update()       │
│    ahrs.get_position()                  │
└─────────────────────────────────────────┘
                    │
┌─────────────────────────────────────────┐
│     AP_AHRS（前端，统一接口）              │
│  ┌─────────────────────────────────┐    │
│  │  AP_AHRS_NavEKF3（适配器/包装器） │    │
│  │  ┌─────────────────────────┐    │    │
│  │  │     NavEKF3（前端）      │    │    │
│  │  │  ┌─────────────────┐    │    │    │
│  │  │  │ NavEKF3_core × N │    │    │    │
│  │  │  │ （后端，实际EKF） │    │    │    │
│  │  │  └─────────────────┘    │    │    │
│  │  └─────────────────────────┘    │    │
│  └─────────────────────────────────┘    │
│  ┌─────────────────────────────────┐    │
│  │  AP_AHRS_NavEKF2（另一个适配器）  │    │
│  │  AP_AHRS_DCM（DCM适配器）        │    │
│  │  AP_AHRS_SIM（仿真适配器）        │    │
│  └─────────────────────────────────┘    │
└─────────────────────────────────────────┘
```

### AP_AHRS_NavEKF3 是什么？

**适配器模式（Adapter/Wrapper）**，位于 `libraries/AP_AHRS/`。

```cpp
// AP_AHRS_NavEKF3.h:19
/*
 *  shim AP_NavEKF3 into AP_AHRS_Backend
 */
```

它的唯一职责：**把 NavEKF3 的接口"翻译"成 AP_AHRS_Backend 的标准接口**。

```cpp
class AP_AHRS_NavEKF3 : public AP_AHRS_Backend {
public:
    void update() override {
        EKF3.UpdateFilter();  // 直接转发给 NavEKF3
    }

    void get_results(Estimates &results) override {
        // 从 NavEKF3 读取数据，填充到 AHRS 统一的 Estimates 结构
    }

    bool get_origin(Location &ret) const override {
        return EKF3.getOriginLLH(ret);  // 转发
    }

    // ... 其他方法都是类似的转发/包装

    static NavEKF3 EKF3;  // 实际的 NavEKF3 实例
};
```

### NavEKF3 是什么？

**真正的 EKF3 实现**，位于 `libraries/AP_NavEKF3/`。

它不关心 AHRS 接口，只专注于：
- 管理多个 `NavEKF3_core`
- 提供 EKF 相关的 API（`getOriginLLH()`、`getVelNED()` 等）

### 为什么要这样分层？

| 层次 | 类 | 职责 |
|---|---|---|
| **AHRS 前端** | `AP_AHRS` | 统一接口，飞行代码只认它 |
| **AHRS 后端** | `AP_AHRS_Backend` | 抽象基类，定义标准接口 |
| **适配器** | `AP_AHRS_NavEKF3` | 把 NavEKF3 包装成 AHRS 后端 |
| **EKF 前端** | `NavEKF3` | 管理多个 EKF core |
| **EKF 后端** | `NavEKF3_core` | 实际跑 EKF 算法 |

**好处：可以切换不同的姿态估计算法**

```cpp
// AP_AHRS.h:77-82
#if AP_AHRS_NAVEKF2_ENABLED
    AP_AHRS_NavEKF2 ekf2;  // EKF2 适配器
#endif
#if AP_AHRS_NAVEKF3_ENABLED
    AP_AHRS_NavEKF3 ekf3;  // EKF3 适配器
#endif
```

通过参数 `AHRS_EKF_TYPE` 选择：
- `0` → DCM
- `2` → EKF2
- `3` → EKF3
- `10` → 仿真

飞行模式代码完全不用改，因为都通过 `AP_AHRS` 统一接口访问。

### 一句话总结

| | |
|---|---|
| `AP_AHRS_NavEKF3` | **适配器**，让 EKF3 能被 AHRS 系统使用 |
| `NavEKF3` | **真正的 EKF3 实现**，做传感器融合 |
| 关系 | `AP_AHRS_NavEKF3` 内部持有 `NavEKF3` 实例，把调用转发给它 |

---

## 十四、命名辨析：NavEKF3 vs EKF3 vs AP_NavEKF3

| 名称 | 实际含义 | 代码中的类/目录 |
|---|---|---|
| **NavEKF3** | 导航扩展卡尔曼滤波器第3版 | `class NavEKF3`（Frontend）<br>`class NavEKF3_core`（Backend）|
| **EKF3** | 扩展卡尔曼滤波器第3版（简称）| 同上，只是简称 |
| **AP_NavEKF3** | ArduPilot 导航 EKF3 库 | 目录名 `libraries/AP_NavEKF3/` |

**本质上：NavEKF3 = EKF3 = AP_NavEKF3**，只是从不同角度称呼：
- **EKF3**：强调算法版本（第3代）
- **NavEKF3**：强调用途（导航）
- **AP_NavEKF3**：强调所属项目（ArduPilot）

---

## 总结

| 问题 | 答案 |
|---|---|
| EKF3 是什么？ | 24 状态扩展卡尔曼滤波器，融合多传感器估计飞行器状态 |
| 数据来源？ | IMU（始终）+ GPS/磁力计/气压计/测距仪/光流/空速/外部导航（aiding 源）|
| 怎么融合？ | IMU 预测 -> 传感器测量修正 -> 卡尔曼增益加权 |
| 输出什么？ | 姿态、速度、位置、传感器零偏、风速、磁场、地形高度 |
| 状态 vs 输出？ | `stateStruct` 在延迟时域做数学，`outputDataNew` 在当前时域给飞控用 |
| 导航 vs 非导航？ | 导航 = 位置/速度/航向；EKF3 同时估计导航和非导航状态 |
| 为什么叫 NavEKF？ | 强调其导航状态估计的核心目的 |
| AHRS 是什么？ | 更高层的抽象接口，可以选择 DCM/EKF2/EKF3 作为后端 |
| AP_AHRS_NavEKF3 是什么？ | AHRS 适配器，把 NavEKF3 包装成标准后端接口 |
| NavEKF3 是什么？ | 真正的 EKF3 实现，做传感器融合 |

---

## 十五、EKF 输出结果代码定位

### 15.1 AHRS 层统一输出（适配器模式）

**文件**: `libraries/AP_AHRS/AP_AHRS_NavEKF3.cpp:13-74`

```cpp
void AP_AHRS_NavEKF3::get_results(AP_AHRS_Backend::Estimates &results)
{
    // === 导航状态输出（姿态）===
    EKF3.getRotationBodyToNED(results.dcm_matrix);   // 方向余弦矩阵
    EKF3.getEulerAngles(eulers);                      // 欧拉角
    EKF3.getQuaternion(results.quaternion);           // 四元数
    results.attitude_valid = started;

    // === 非导航状态输出（传感器偏差）===
    EKF3.getGyroBias(-1, drift);                      // 陀螺零偏
    results.gyro_drift = -drift;
    EKF3.getAccelBias(-1, results.accel_bias);        // 加速度计零偏

    // === 导航状态输出（速度）===
    EKF3.getVelNED(results.velocity_NED);             // NED速度
    results.velocity_NED_valid = true;
    results.vert_pos_rate_D = EKF3.getPosDownDerivative();
    results.vert_pos_rate_D_valid = true;

    // === 导航状态输出（位置）===
    results.location_valid = EKF3.getLLH(results.location);  // 经纬高位置
}
```

### 15.2 EKF3 核心层输出（实际实现）

**文件**: `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp`

#### 导航状态输出（直接来自 `outputDataNew`，当前时域）

| 函数 | 行号 | 数据来源 |
|------|------|----------|
| `getEulerAngles()` | L126 | `outputDataNew.quat.to_euler()` |
| `getRotationBodyToNED()` | L153 | `outputDataNew.quat.rotation_matrix()` |
| `getQuaternion()` | L160 | `outputDataNew.quat` |
| `getVelNED()` | L209 | `outputDataNew.velocity + velOffsetNED` |
| `getPosNE()` | L257 | `outputDataNew.position.xy()` |
| `getPosD_local()` | L298 | `outputDataNew.position.z + posOffsetNED.z` |
| `getLLH()` | L334 | `outputDataNew.position` + origin |

#### 非导航状态输出（来自 `stateStruct`，延迟时域）

| 函数 | 行号 | 数据来源 |
|------|------|----------|
| `getGyroBias()` | L133 | `stateStruct.gyro_bias / dtEkfAvg` |
| `getAccelBias()` | L143 | `stateStruct.accel_bias / dtEkfAvg` |
| `getWind()` | L199 | `stateStruct.wind_vel.x/y` |
| `getMagNED()` | L428 | `stateStruct.earth_magfield * 1000` |
| `getMagXYZ()` | L434 | `stateStruct.body_magfield * 1000` |

### 15.3 关键数据结构定义

**24 状态结构体**（`AP_NavEKF3_core.h:574-588`）：

```cpp
struct state_elements {
    QuaternionF quat;           // 0-3: 姿态（导航状态）
    Vector3F    velocity;       // 4-6: 速度（导航状态）
    Vector3F    position;       // 7-9: 位置（导航状态）
    Vector3F    gyro_bias;      // 10-12: 陀螺零偏（非导航状态）
    Vector3F    accel_bias;     // 13-15: 加速度计零偏（非导航状态）
    Vector3F    earth_magfield; // 16-18: 地磁场（非导航状态）
    Vector3F    body_magfield;  // 19-21: 机体磁场（非导航状态）
    Vector2F    wind_vel;       // 22-23: 风速（非导航状态）
};
```

**输出观测器结构体**（`AP_NavEKF3_core.h:590-594`）：

```cpp
struct output_elements {
    QuaternionF quat;           // 姿态
    Vector3F    velocity;       // 速度（已补偿到机体原点）
    Vector3F    position;       // 位置（已补偿到机体原点）
};
```

### 15.4 核心区别

| 特性 | `stateStruct`（24状态） | `outputDataNew`（输出状态） |
|------|------------------------|----------------------------|
| **时域** | 延迟时域（fusion time horizon） | 当前时域（current time） |
| **位置参考点** | IMU 位置 | 机体原点（body frame origin） |
| **用途** | EKF 内部融合计算 | 外部输出给控制回路 |
| **包含内容** | 24状态（导航+非导航） | 仅导航状态（姿态/速度/位置） |
| **补偿** | 无 IMU 偏移补偿 | 有 `velOffsetNED`/`posOffsetNED` 补偿 |

### 15.5 AHRS 主控调用流程

**文件**: `libraries/AP_AHRS/AP_AHRS.cpp:701-751`

```cpp
void AP_AHRS::update_EKF3(void)
{
    if (ekf3.started) {
        ekf3.update();                          // 1. 执行 EKF 预测+融合
        ekf3_estimates = {};
        ekf3.get_results(ekf3_estimates);       // 2. 获取 EKF 输出结果
        if (_active_EKF_type() == EKFType::THREE) {
            copy_estimates_from_backend_estimates(ekf3_estimates);  // 3. 复制到 AHRS 统一状态
        }
    }
}
```

### 15.6 总结

所有 EKF 状态（导航+非导航）都在 **NavEKF3_core** 的 `stateStruct` 中估计和维护。输出时：

- **导航状态**通过**输出观测器** (`outputDataNew`) extrapolate 到当前时刻，并补偿 IMU 偏移
- **非导航状态**直接从 `stateStruct` 读取

AHRS 层通过 `AP_AHRS_NavEKF3` 适配器统一封装后提供给飞控使用。
