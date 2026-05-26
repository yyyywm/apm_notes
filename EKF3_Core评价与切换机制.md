# EKF3 Core 评价与切换机制

## 一、背景：为什么有多个 Core？

EKF3 支持最多 3 个并行运行的 EKF 实例（core），`MAX_EKF_CORES = 3`。每个 core 通常绑定到不同的 IMU（由 `EK3_IMU_MASK` 控制），各自独立运行完整的滤波器。

多 core 的目的是**冗余**——当某个 IMU 出现故障或性能下降时，飞控可以切换到另一个 core，避免触发 EKF failsafe。

---

## 二、每个 Core 的六维评价画像

每个 core 在每一时刻由以下 6 个维度刻画：

### 2.1 `errorScore()` — 瞬时融合质量分数

**文件：** `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp:62-86`

**分数越高 = 表现越差。** 前提：`tiltAlignComplete && yawAlignComplete` 都完成后才计算，否则返回 0。

```python
score = 0.0

# 1. GPS 位置+速度 — 权重 0.5
score = max(score, 0.5 × (velTestRatio + posTestRatio))

# 2. 高度 — 权重 1.0（影响最大）
score = max(score, hgtTestRatio)

# 3. 空速 — 权重 0.3（条件：固定翼 + ≥2个空速传感器 + EKF_AFFINITY_ARSP）
score = max(score, 0.3 × tasTestRatio)

# 4. 磁力计 — 权重 0.3（条件：EKF_AFFINITY_MAG）
score = max(score, 0.3 × (magTestRatio.x + magTestRatio.y + magTestRatio.z))
```

取 `MAX()` 意味着**短板效应**——哪个传感器最差，分数就是多少。

### 2.2 `healthy()` — 健康状态

**文件：** `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp:9-33`

任一条件满足即**不健康**：

| 条件 | 含义 |
|---|---|
| `getFilterFaults() != 0` | 8 种底层故障之一（见下文） |
| `velTestRatio > 1 && posTestRatio > 1 && hgtTestRatio > 1` | 三个核心指标**同时**超出 gate |
| 运行时间 < 1 秒 | 滤波器尚未稳定 |
| 地面 + 无外部辅助 + 位置漂移 > 1m | 地面静止时不应漂移 |

#### `getFilterFaults()` — 8 种底层故障（位掩码）

**文件：** `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp:589-598`

| 位 | 故障 | 含义 |
|---|---|---|
| 0 | 四元数为 NaN | 姿态解算崩溃 |
| 1 | 速度为 NaN | 速度估计崩溃 |
| 2 | X 轴磁力计融合病态 | 协方差矩阵奇异，无法融合 |
| 3 | Y 轴磁力计融合病态 | 同上 |
| 4 | Z 轴磁力计融合病态 | 同上 |
| 5 | 空速融合病态 | 空速协方差矩阵奇异 |
| 6 | 侧滑融合病态 | 侧滑角协方差矩阵奇异 |
| 7 | 滤波器未初始化 | `!statesInitialised` |

### 2.3 `tiltAlignComplete` — 倾角对齐状态

EKF 初始化过程中，需要先完成倾角（roll/pitch）估计。只有完成了 tilt 对齐，滤波器才知道"水平面"在哪里。未完成对齐的 core 不能作为 primary。

### 2.4 `yawAlignComplete` — 偏航对齐状态

倾角对齐之后的第二步骤。需要磁力计数据或 GPS 航向等来源来确定绝对朝向。未完成 yaw 对齐的 core 在正常路径中可以当 primary（只要 tilt 对齐了），但在故障安全路径中不能。

### 2.5 `coreRelativeErrors[i]` — 累积相对误差

**文件：** `libraries/AP_NavEKF3/AP_NavEKF3.cpp:1105-1118`

这是**长期历史表现**的度量，只对备用 core 计算：

```python
for each 非 primary 的 core i:
    error = coreErrorScores[i] - coreErrorScores[primary]
    # 正数 = 比 primary 差，负数 = 比 primary 好

    # 只有超过阈值的差异才累积
    if error > 0 or error < -MAX(_err_thresh, 0.05):
        coreRelativeErrors[i] += error
        coreRelativeErrors[i] = clamp(coreRelativeErrors[i], -1.0, 1.0)
```

关键设计：
- **正误差（比 primary 差）永远累积**，不设门槛
- **负误差（比 primary 好）只有改善幅度 > `_err_thresh`（默认 0.2）时才累积**，防止噪声引发误切换
- 夹在 `[-1.0, 1.0]`（`CORE_ERR_LIM = 1.0`）
- 每次切换 primary 后归零

### 2.6 退避时间

每个 core 记录最后一次当 primary 的时间 `coreLastTimePrimary_us[i]`。刚被换下的 core **至少 10 秒内**不能再次当选，防止两个 core 来回震荡。

---

## 三、EKF 底层：innovation、gate 与 testRatio

### 3.0 EKF 的工作原理（极简版）

EKF 做两件事，交替进行：

```
┌─────────────────────────────────────────────────────┐
│  PREDICT（预测）                                     │
│  ┌─────────────────────────────────────────────┐    │
│  │ 用 IMU 数据"推"状态：                         │    │
│  │   姿态 += 陀螺仪积分                          │    │
│  │   速度 += (加速度 - 重力) × dt                │    │
│  │   位置 += 速度 × dt                          │    │
│  │                                             │    │
│  │ 同时，协方差矩阵 P 增大（不确定性增加）         │    │
│  │   P = F × P × Fᵀ + Q                        │    │
│  │   (状态转移 + 过程噪声)                       │    │
│  └─────────────────────────────────────────────┘    │
│                       │                             │
│                       ▼                             │
│  UPDATE（更新/融合）                                  │
│  ┌─────────────────────────────────────────────┐    │
│  │ 传感器数据到达时：                            │    │
│  │                                             │    │
│  │ ① 计算 innovation = 预测值 - 测量值           │    │
│  │    预测值 = H(state)  ← "滤波器认为传感器该看到什么"│    │
│  │    测量值 = sensor  ← "传感器实际看到了什么"   │    │
│  │                                             │    │
│  │ ② 计算 innovation 方差 = H×P×Hᵀ + R          │    │
│  │    H×P×Hᵀ = 状态不确定性投影到测量空间         │    │
│  │    R = 传感器自身噪声（观测噪声）               │    │
│  │                                             │    │
│  │ ③ 一致性检验：testRatio = innov² / (gate² × var)│
│  │    通过 → 用这个测量值修正状态                 │    │
│  │    不通过 → 丢弃这个测量（可能是野值）          │    │
│  │                                             │    │
│  │ ④ 卡尔曼增益 K = P×Hᵀ / var                  │    │
│  │    state += K × innovation                   │    │
│  │    P -= K × H × P                           │    │
│  └─────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

### 3.1 什么是 innovation？

**innovation = state_predicted - sensor_measured**

物理含义：**滤波器对自己"被打脸"程度的度量。**

| innovation 值 | 含义 |
|---|---|
| 接近 0 | 滤波器预测和传感器一致，没有意外 |
| 正大 | 滤波器预测比传感器大很多——可能传感器偏低，或滤波器状态漂高了 |
| 负大 | 传感器比滤波器预测大很多——可能传感器偏高，或滤波器状态漂低了 |

例子：GPS 报速度 10 m/s，EKF 从 IMU 推算预测的速度是 12 m/s → `vel_innovation = 12 - 10 = 2 m/s`。滤波器会部分采信这个差异，把速度估计往下拉一点（拉多少取决于卡尔曼增益 K）。

### 3.2 什么是 innovation 方差（varInnov）？

**varInnov = H × P × Hᵀ + R**

| 项 | 含义 | 来源 |
|---|---|---|
| `H × P × Hᵀ` | 状态协方差投影到观测空间 | P 矩阵（24×24 协方差矩阵，EKF 内部维护） |
| `R` | 传感器自身测量噪声 | 传感器噪声模型（如 `R_OBS_DATA_CHECKS[i]`） |

**为什么需要它？** 因为 innovation 不是绝对值，要看它"相对其自身不确定性"有多大。

```
场景 A: innovation = 2 m/s, 方差 = 4  → testRatio = 4/(25×4) = 0.04 ← 正常
场景 B: innovation = 2 m/s, 方差 = 0.01 → testRatio = 4/(25×0.01) = 16 ← 严重！
```

同样的 2 m/s 差异，如果传感器的噪声只有 0.1 m/s²，那这个差异就是 16 个标准差——几乎肯定是测量出了问题。

### 3.3 什么是 gate？

**gate = 允许的 σ（标准差）倍数。** 默认值通常为 5（即 5σ）。

```
testRatio = innovation² / (gate² × varInnov)
         = (innovation / (gate × sqrt(varInnov)))²
         = (innovation / gate / σ_innovation)²
```

所以 `testRatio > 1` 等价于 `|innovation| > gate × σ_innovation`，即 innovation 超出了 gate 倍的标准差。

Gate 越大 → 越宽容（接受更多测量），Gate 越小 → 越严格（拒绝更多测量）。

| 参数 | 默认 gate | 含义 |
|---|---|---|
| `EK3_VEL_I_GATE` | 5.0σ | 速度 innovation 允许 5 倍标准差 |
| `EK3_POS_I_GATE` | 5.0σ | 位置 innovation 允许 5 倍标准差 |
| `EK3_HGT_I_GATE` | 5.0σ | 高度 innovation 允许 5 倍标准差 |
| `EK3_MAG_I_GATE` | 3.0σ | 磁力计 innovation 允许 3 倍标准差 |
| `EK3_TAS_I_GATE` | 5.0σ | 空速 innovation 允许 5 倍标准差 |

### 3.4 testRatio 的完整数据流

```
IMU 数据到达
    │
    ▼
PREDICT（状态预测 + 协方差增长）
    │
    ▼
传感器数据到达（GPS/气压计/磁力计等）
    │
    ▼
① 计算预测值
   state → 通过观测模型 H(state) → "你该看到什么"
    │
    ▼
② innovation = 预测值 - 传感器实际值
    │
    ▼
③ varInnov = H×P×Hᵀ + R
   （P 是 24×24 协方差矩阵，内部记录着状态的不确定性）
    │
    ▼
④ testRatio = innovation² / (gate² × varInnov)
    │
    ├── testRatio < 1 → 通过！用这个测量值更新状态
    │       state += K × innovation
    │       P -= K × H × P
    │
    └── testRatio ≥ 1 → 失败！丢弃这个测量
            (除非超时或 bad IMU，会强制融合)
```

**一个具体例子：**

假设飞机在平飞，EKF 从 IMU 推算速度是 20 m/s。GPS 报速度 18 m/s。EKF 当前对速度的协方差（P 矩阵对角线）大约是 1.0 m²/s²，GPS 噪声 R 约 0.5 m²/s²。

```
innovation = 20 - 18 = 2 m/s

varInnov = H×P×Hᵀ + R = 1.0 + 0.5 = 1.5 m²/s²

testRatio = 2² / (5² × 1.5) = 4 / 37.5 = 0.107

0.107 < 1 → 通过检验 → 用这个测量值更新状态
```

如果 GPS 突然报 50 m/s（可能是多径效应）：

```
innovation = 20 - 50 = -30 m/s
testRatio = 900 / 37.5 = 24.0

24.0 >> 1 → 拒绝！这个 GPS 数据被当作野值丢弃
```

### 3.5 各传感器 testRatio 的具体公式

所有公式展开为 `testRatio = innovation² / (gate² × varInnov)`。

但关键在于 **innovation 从哪里来、varInnov 由什么构成、gate 参数是哪个**——每个传感器都不同。

---

#### 3.5.1 `posTestRatio` — GPS 位置

**文件：** `AP_NavEKF3_PosVelFusion.cpp:915-926`

```cpp
// innovation = EKF 状态 - GPS 测量值
innovVelPos[3] = stateStruct.position.x - velPosObs[3];  // 北向 (m)
innovVelPos[4] = stateStruct.position.y - velPosObs[4];  // 东向 (m)

// varInnov = 状态协方差对角线 + 观测噪声（GPS 的位置精度）
varInnovVelPos[3] = P[7][7] + R_OBS_DATA_CHECKS[3];
varInnovVelPos[4] = P[8][8] + R_OBS_DATA_CHECKS[4];
//  P[7][7] = 北向位置状态的协方差（状态 7 = position.x）
//  P[8][8] = 东向位置状态的协方差（状态 8 = position.y）
//  R_OBS_DATA_CHECKS[3/4] = GPS 报告的水平位置噪声方差 (m²)

// gate 参数
ftype maxPosInnov2 = sq(MAX(0.01 * _gpsPosInnovGate, 1.0))
                   * (varInnovVelPos[3] + varInnovVelPos[4]);
// _gpsPosInnovGate = EK3_POS_I_GATE，默认 500 (即 5.0σ)

posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
```

北向和东向的 innovation 平方和**合并**为**一个**比值（不是两个独立比值）。

---

#### 3.5.2 `velTestRatio` — GPS 速度

**文件：** `AP_NavEKF3_PosVelFusion.cpp:982-1001`

```cpp
uint8_t imax = 2;  // 默认融合水平两轴 (N/E)，若有 GPS 垂速则融三轴
// 注意：非绝对 mode 或不融 GPS 垂速时 imax=1（只融水平）

ftype innovVelSumSq = 0;
ftype varVelSum = 0;

for (uint8_t i = 0; i <= imax; i++) {
    stateIndex = i + 4;  // 状态索引 4/5/6 = velocity.x/y/z

    // innovation = EKF 速度状态 - GPS 速度测量
    innovation = stateStruct.velocity[i] - velPosObs[i];
    innovVelSumSq += sq(innovation);

    // varInnov = P_{速度,速度} + GPS 速度噪声
    varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
    varVelSum += varInnovVelPos[i];
}

velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01 * _gpsVelInnovGate, 1.0)));
// _gpsVelInnovGate = EK3_VEL_I_GATE，默认 500 (即 5.0σ)
```

所有轴合并成一个比值。水平速度用 P[4][4] 和 P[5][5]，垂直速度用 P[6][6]。

---

#### 3.5.3 `hgtTestRatio` — 高度

**文件：** `AP_NavEKF3_PosVelFusion.cpp:1040-1044`

```cpp
// innovation = EKF 高度 - 高度传感器（气压计/GPS/测距仪）
innovVelPos[5] = stateStruct.position.z - velPosObs[5];  // (m)

// varInnov = 高度协方差 + 高度传感器噪声
varInnovVelPos[5] = P[9][9] + R_OBS_DATA_CHECKS[5];
//  P[9][9] = D 轴位置状态的协方差（状态 9 = position.z）
//  R_OBS_DATA_CHECKS[5] = 高度传感器的噪声方差 (m²)

hgtTestRatio = sq(innovVelPos[5])
             / (sq(MAX(0.01 * _hgtInnovGate, 1.0)) * varInnovVelPos[5]);
// _hgtInnovGate = EK3_HGT_I_GATE，默认 500 (即 5.0σ)
```

单轴，不合并。**在 errorScore 中权重为 1.0，是影响最大的分量。**

---

#### 3.5.4 `tasTestRatio` — 真空速

**文件：** `AP_NavEKF3_AirDataFusion.cpp:102-105`

```cpp
// 观测方程: V_tas = sqrt(v_rel_x² + v_rel_y² + v_rel_z²)
// 其中 v_rel = wind_velocity - groundspeed
// innovVtas = 预测空速 - 测量空速 (m/s)

// varInnov 通过卡尔曼滤波的标准公式计算:
// temp = R_TAS + H×P×Hᵀ  (一个涉及 P[4][4], P[5][5], P[6][6], P[22][22], P[23][23] 的复杂表达式)
SK_TAS[0] = 1.0 / temp;
varInnovVtas = 1.0 / SK_TAS[0];  // 即 temp

tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * _tasInnovGate, 1.0f)) * varInnovVtas);
// _tasInnovGate = EK3_TAS_I_GATE，默认 500 (即 5.0σ)
```

与 GPS/高度不同，`varInnovVtas` 不是简单的 P[i][i] + R，而是涉及**完整的 H×P×Hᵀ 投影**（因为空速的观测方程是非线性的，涉及风速状态 22/23、速度状态 4/5/6 的交叉项）。

---

#### 3.5.5 `magTestRatio` — 磁力计（三轴独立）

**文件：** `AP_NavEKF3_MagFusion.cpp:535-573`

```cpp
// X 轴
innovMag[0] = 预测磁场 - 测量磁场;  // (milligauss)
varInnovMag[0] = P[19][19] + R_MAG + [涉及 P[1][19], P[2][19], P[3][19],
                  P[16][19], P[17][19], P[18][19], P[0][19] 的交叉项];
//  P[19][19] = body_magfield.x 状态的协方差
//  P[16][16] = earth_magfield.x 状态的协方差 (地磁场)
//  P[1/2/3] = 四元数状态
//  R_MAG = 磁力计观测噪声

// Y 轴 — 类似，P[20][20], P[17][17]
varInnovMag[1] = P[20][20] + R_MAG + [...];

// Z 轴 — 类似，P[21][21], P[18][18]
varInnovMag[2] = P[21][21] + R_MAG + [...];

for (uint8_t i = 0; i <= 2; i++) {
    magTestRatio[i] = sq(innovMag[i])
                    / (sq(MAX(0.01f * _magInnovGate, 1.0f)) * varInnovMag[i]);
}
// _magInnovGate = EK3_MAG_I_GATE，默认 300 (即 3.0σ)
// ★ 磁力计的 gate 默认 3σ，比其他传感器（5σ）更严格
```

**X/Y/Z 三轴分别计算，不合并。** 磁力计的 innovation 方差涉及**姿态四元数状态 (P[0]-P[3])、地磁场状态 (P[16]-P[18])、机体磁场状态 (P[19]-P[21]) 的交叉协方差**，是所有 testRatio 中最复杂的一个。

---

### 3.6 总结：P 矩阵与 testRatio 的对应关系

EKF 维护 **24×24 协方差矩阵 P**，每个对角线项代表对应状态的不确定性：

| 状态 | 索引 | P 索引 | 影响哪个 testRatio |
|---|---|---|---|
| quat.w/x/y/z | 0-3 | P[0][0]-P[3][3] | mag（通过交叉项） |
| velocity N/E/D | 4-6 | P[4][4]-P[6][6] | vel, tas |
| position N/E/D | 7-9 | P[7][7]-P[9][9] | pos, hgt |
| gyro_bias | 10-12 | P[10][10]-P[12][12] | 间接（通过预测步骤） |
| accel_bias | 13-15 | P[13][15]-P[15][15] | 间接（通过预测步骤） |
| earth_magfield | 16-18 | P[16][16]-P[18][18] | mag（核心） |
| body_magfield | 19-21 | P[19][19]-P[21][21] | mag（核心） |
| wind_vel N/E | 22-23 | P[22][22]-P[23][23] | tas（核心） |

每种传感器融合只用到 P 矩阵中与自己相关的子块。GPS 位置只看 P[7][7] 和 P[8][8]，磁力计则涉及 4 个状态群组的交叉协方差。

---

## 四、Core 选择的两条路径

EKF3 的 core 选择有**两条独立的路径**，运行时机和判断逻辑不同：

| | 正常运行路径 | 故障安全路径 |
|---|---|---|
| 触发时机 | 每次 `UpdateFilter()`，已解锁时 | 车辆即将触发 EKF failsafe 时（fail_count=9） |
| 比较依据 | 累积相对误差 `coreRelativeErrors[]` | 瞬时误差 `errorScore()` |
| 切换门槛 | 累积误差 ≤ -0.5 或 primary 不健康 | errorScore < 0.9 且低于当前 primary |
| 限速 | 10 秒退避 | 5 秒内最多切一次 |

---

## 五、路径一：正常运行时切换（UpdateFilter）

**文件：** `libraries/AP_NavEKF3/AP_NavEKF3.cpp:933-1005`

### 执行流程：Primary vs 备用 Core 分开看

每一步 `UpdateFilter()` 中，primary 和备用 core 的角色不同：

---

#### Step 0 — 门控（仅 primary 被检查）

```cpp
if (!runCoreSelection):
    检查 core[primary].healthy()  // ← 只有 primary 被查
    必须连续健康 10 秒，runCoreSelection 才变成 true

if 未解锁:
    primary = user_primary  // 强制回到用户设定
    return
```

| | Primary | 备用 Core |
|---|---|---|
| 动作 | 被查 `healthy()`，等 10 秒 | 无事 |

---

#### Step 1 — `updateCoreErrorScores()`：计算瞬时分数（所有 core 都参与）

```cpp
for (uint8_t i = 0; i < num_cores; i++) {
    coreErrorScores[i] = core[i].errorScore();
}
return coreErrorScores[primary];
```

| | Primary | 备用 Core |
|---|---|---|
| 动作 | 调 `errorScore()`，结果存 `coreErrorScores[primary]` | 调 `errorScore()`，结果存 `coreErrorScores[i]` |
| 前提 | `tiltAlignComplete && yawAlignComplete`，否则返回 0 | 同左 |
| 算完用途 | ① 作为基准线（Step 2 减法）② 作为绝对阈值（Step 4：`>1.0` 即触发） | 作为差值来源（Step 2） |

---

#### Step 2 — `updateCoreRelativeErrors()`：更新累积相对误差（仅备用 core 参与）

```cpp
for (uint8_t i = 0; i < num_cores; i++) {
    if (i != primary) {  // ← primary 不参与
        error = coreErrorScores[i] - coreErrorScores[primary];
        // 正误差永远累积，负误差需改善 > _err_thresh(0.2) 才累积
        if (error > 0 || error < -MAX(_err_thresh, 0.05)) {
            coreRelativeErrors[i] += error;
            coreRelativeErrors[i] = clamp(coreRelativeErrors[i], -1.0, 1.0);
        }
    }
}
```

| | Primary | 备用 Core |
|---|---|---|
| 动作 | **不参与**。它是基准线（零点），不需要跟自己比 | 用 `errorScore[i] - errorScore[primary]` 累积 |
| 门槛 | — | 正误差永远累积；负误差需改善幅度 > 0.2 才累积 |
| 范围 | — | 夹在 `[-1.0, 1.0]` |

---

#### Step 3 — 寻找最佳候选（仅备用 core 参与）

```cpp
uint8_t newPrimaryIndex = primary;  // 初始 = primary 自己

for (uint8_t coreIndex = 0; coreIndex < num_cores; coreIndex++) {
    if (coreIndex != primary) {  // ← primary 不参与遍历
```

对每个备用 core 依次进行三重筛选：

**3a: `coreBetterScore()` — 两两淘汰赛**

```cpp
bool NavEKF3::coreBetterScore(uint8_t new_core, uint8_t current_core) const
{
    if (!newCore.healthy())     { return false; }  // ① 不健康→直接淘汰
    if (tilt 对齐不同)           { return newCore 对齐了; }  // ② tilt 优先
    if (yaw 对齐不同)            { return newCore 对齐了; }  // ③ yaw 次之
    return coreRelativeErrors[new] < coreRelativeErrors[current];  // ④ 比累积误差
}
```

注意 `current_core` 初始是 primary，但如果之前的备用 core 已胜出，则 `current_core` 变为那个备用 core——这是**备选之间的淘汰赛**。

**3b: 退避时间**

```cpp
imuSampleTime_us - coreLastTimePrimary_us[i] > 1E7  // 10 秒
```

刚被换下的 core 至少 10 秒内不能再当选，防止来回震荡。

**3c: `betterCore` 判定**

```cpp
betterCore = coreRelativeErrors[i] <= -0.5;  // 累积误差显著优于 primary
// 或：备用有 yaw 对齐而 primary 没有
betterCore |= newCore.have_aligned_yaw() && !oldCore.have_aligned_yaw();
```

| | Primary | 备用 Core |
|---|---|---|
| 被检查项 | 作为比较对象参与 `coreBetterScore` | `healthy()` / 对齐 / 累积误差 / 退避时间 |
| 角色 | **守擂方** | **挑战者**，需通过全部筛选 |

---

#### Step 4 — 切换判定（primary 被审判）

```cpp
altCoreAvailable = (newPrimaryIndex != primary);  // 有合格的备用吗？

if (altCoreAvailable && (
    primaryErrorScore > 1.0f        // ← primary 的瞬时分数
    || !core[primary].healthy()     // ← primary 的健康状态
    || betterCore                   // ← 备用累积误差 ≤ -0.5
)) {
    switchLane(newPrimaryIndex);    // 切到胜出的备用 core
    resetCoreErrors();              // 所有累积误差归零
    coreLastTimePrimary_us[old] = imuSampleTime_us;  // 记录退避时间
}
```

| | Primary | 备用 Core |
|---|---|---|
| 被检查什么 | `errorScore > 1.0`? `healthy()`? | `betterCore` 为 true?（挑战是否成立） |
| 角色 | **被审判者**：三个条件满足任一就被换下 | **挑战者**：挑战成功 → 晋升为 primary，累积误差归零 |
| 切换后 | 记录退避时间，10 秒内不能再当选 | 成为新 primary |

---

### 小结：谁在每一步做了什么

```
Step 0  门控        primary: 被查 healthy，等 10 秒
                    备用:   无事

Step 1  errorScore  所有 core 都算，primary 的结果同时作为"基准"和"绝对阈值"

Step 2  累积误差     primary: 不参与（它是零点）
                    备用:   差值累加到 coreRelativeErrors

Step 3  找候选       primary: 作为比较对象（coreBetterScore 的 current_core）
                    备用:   接受 healthy/tilt/yaw/累积误差/退避 五重筛选

Step 4  切换         primary: 被审判 — errorScore>1.0 / !healthy / betterCore
                    备用:   挑战成功 → 晋升为 primary，累积误差归零
```

### 关键设计：不对称性

```
从 primary 切走：容易（瞬时判断）
切到新 primary：  困难（需要累积证据）

原因：防止振荡。
- primary 出问题时快速切走 → 保证安全（errorScore>1.0 或 !healthy 瞬时触发）
- 备用 core 要取代健康的 primary 需要长期证明 → 保证稳定（累积误差 ≤ -0.5）
- 退避 10 秒 → 进一步防止来回切换
```

### 找不到备用 core 时的兜底逻辑

如果 Step 3 遍历完所有备用 core，`newPrimaryIndex` 仍然是 `primary`（没有合格的挑战者），那么：

```
altCoreAvailable == false
```

此时**即使 `primaryErrorScore > 1.0` 或 `!healthy()`，也不会切换**——因为没有合格的接班人。

这种情况下，primary 继续"带病上岗"，而车辆层面的 `ekf_check`（10Hz）会继续累积 `fail_count`：

```
fail_count 递增 → 到 8 时尝试 request_yaw_reset()
                → 到 9 时尝试 check_lane_switch()（故障安全路径，门槛不同）
                → 到 10 时正式触发 EKF failsafe（降落/定高）
```

也就是说：

| 场景 | 结果 |
|---|---|
| primary 差 + 有合格备用 | Step 4 直接切，飞行继续 |
| primary 差 + 无合格备用 | 硬扛，等 ekf_check 走到 fail_count=10 → failsafe |
| primary 差 + 短暂变差后恢复 | `fail_count` 开始递减，回到 0，不触发 failsafe |

**故障安全路径的 `checkLaneSwitch()` 比正常路径更激进**——它不要求累积误差，直接用瞬时 `errorScore()` 比较，且只要求备用 `errorScore < 0.9`。这意味着正常路径找不到的备用 core，故障路径有可能找到（因为不再需要累积误差达标）。

### 通道切换条件总结

两条路径汇总：

| 触发条件 | 正常路径 (`UpdateFilter`) | 故障安全路径 (`checkLaneSwitch`) |
|---|---|---|
| 调用时机 | 每次 `UpdateFilter()`，已解锁时 | `ekf_check` 的 `fail_count=9` 时 |
| primary `errorScore > 1.0` | **触发** | — |
| primary `!healthy()` | **触发** | — |
| 备用累积误差 ≤ -0.5 | **触发** | — |
| 仅 primary `errorScore > 1.0` 但无合格备用 | **不触发**（硬扛） | — |
| primary 健康 且 无 `betterCore` | **不触发** | — |
| 备用 `healthy()` + yaw对齐 + tilt对齐 + `errorScore < 0.9` 且最低 | — | **触发** |
| 限速 | 退避 10 秒 | 5 秒内最多一次 |
| 比较依据 | 累积相对误差 | 瞬时 `errorScore()` |

**兜底链条：**

```
无合格备用 (altCoreAvailable=false) → 硬扛
    ↓
ekf_check fail_count 递增
    ↓
count=8: request_yaw_reset()（重置 yaw，可能恢复 primary）
    ↓
count=9: check_lane_switch()（故障路径，门槛更低）
    ↓
count=10: EKF failsafe（降落/定高）
```

### Disarmed（未解锁）行为

当**未解锁且在地面**时，primary 始终强制回到用户设定的 `EK3_PRIMARY` 参数值。这避免了不同飞行架次间因 GPS 时序抖动导致选择不同 IMU 的问题。

---

## 六、路径二：故障安全切换（checkLaneSwitch）

**文件：** `libraries/AP_NavEKF3/AP_NavEKF3.cpp:1029-1061`

当车辆层面的 EKF failsafe 即将触发（fail_count = 9/10）时调用，策略更激进：

```
限速检查：5 秒内最多执行一次

对每个备用 core：
    ① newCore.healthy()          ← 必须健康
    ② newCore.have_aligned_yaw() ← 必须 yaw 对齐
    ③ newCore.have_aligned_tilt()← 必须 tilt 对齐
    ④ altErrorScore < 0.9        ← 分数必须 < 0.9（比正常路径的 1.0 更严）
    ⑤ altErrorScore < lowestErrorScore ← 取最低分

直接切换到找到的最佳 core（不等待累积误差）
```

这条路径**不使用累积误差**，直接比瞬时 `errorScore()`——因为此时情况紧急，不能再慢慢积累证据。

---

## 七、车辆层面的 EKF Failsafe

**文件：** `ArduCopter/ekf_check.cpp`（以 Copter 为例）

10Hz 频率运行，数据来自 primary core 的 testRatio 的平方根：

```python
vel_var      = sqrt(velTestRatio)   # "速度方差"
pos_var      = sqrt(posTestRatio)   # "位置方差"
mag_max      = max(sqrt(magTestRatio.x), sqrt(magTestRatio.y), sqrt(magTestRatio.z))
```

### 投票机制 `ekf_over_threshold()`

```python
over_thresh_count = 0

if mag_max >= fs_ekf_thresh:          # 磁力计超限 → +1 票
    over_thresh_count += 1

if vel_var >= 2.0 * fs_ekf_thresh:    # 速度严重超限(2倍) → +2 票
    over_thresh_count += 2
elif vel_var >= fs_ekf_thresh:        # 速度超限 → +1 票
    over_thresh_count += 1

# 判定：至少两票（两个传感器都在告警）
if (pos_var >= fs_ekf_thresh and over_thresh_count >= 1) \
   or over_thresh_count >= 2:
    return True  # 触发！
```

### 完整时间线

```
0.0s  ────────────────────────────────────────────────────────
      ekf_check 每 0.1s 运行一次
      获取 primary core 的 variances（低通滤波后）
      投票判定是否超限

0.1s  第一次不通过 → fail_count = 1
0.2s  fail_count = 2
...
0.7s  fail_count = 7
0.8s  fail_count = 8 → ahrs.request_yaw_reset()
      │  尝试重置 primary core 的 yaw
      │  适用于磁力计干扰导致的 yaw 漂移
0.9s  fail_count = 9 → ahrs.check_lane_switch()
      │  尝试切换到更好的 EKF core (路径二)
      │  如果找到: 切换 → variance 下降 → fail_count 递减 → 解除
1.0s  fail_count = 10 → bad_variance = true
      └→ failsafe_ekf_event()  正式触发！
          ├─ REPORT_ONLY: 只记录不干预
          ├─ ALTHOLD: 切到定高模式（或降落）
          ├─ LAND: 降落
          └─ LAND_EVEN_STABILIZE: 即使当前模式不需要位置也降落

如果中间任何一次检查通过 → fail_count-- → 回到 0 → 解除
```

### Failsafe 与 healthy() 的关系

**Failsafe 可以在 primary core 自认为"健康"的情况下触发。** 因为：
- `healthy()` 要求三个 testRatio **同时 > 1** 或存在 filter fault
- Failsafe 的 `FS_EKF_THRESH` 典型值仅为 0.5-0.8，比 1.0 更严格
- Failsafe 只需要**两个传感器**超限，不需要三个

---

## 八、完整决策流程图

```
                    启动/上电
                       │
                       ▼
              ┌─────────────────┐
              │ 设置 primary =   │
              │ 用户 EK3_PRIMARY │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │ 等待 primary 健康 │
              │  稳定 ≥ 10 秒    │
              └────────┬────────┘
                       │
                       ▼
       ┌───────────────────────────────┐
       │       已解锁 (armed)?         │
       ├───────────────────────────────┤
       │ NO ──► 强制使用 EK3_PRIMARY   │
       │         (避免架次间差异)       │
       ├───────────────────────────────┤
       │ YES ──► 正常运行 core 选择逻辑 │
       └───────────────────────────────┘
                       │
           ┌───────────┴───────────┐
           │                       │
           ▼                       ▼
   ┌───────────────┐       ┌───────────────┐
   │ 每次 UpdateFilter│       │ ekf_check 10Hz │
   │ (正常路径)       │       │ (车辆层面)      │
   └───────┬───────┘       └───────┬───────┘
           │                       │
           ▼                       ▼
   ┌───────────────┐       ┌───────────────┐
   │ 计算各 core 的  │       │ 获取 primary   │
   │ errorScore()   │       │ 的 variances   │
   └───────┬───────┘       └───────┬───────┘
           │                       │
           ▼                       ▼
   ┌───────────────┐       ┌───────────────┐
   │ 更新累积相对误差 │       │ 投票判定是否超限 │
   │ RelativeErrors │       │ over_threshold │
   └───────┬───────┘       └───────┬───────┘
           │                       │
           ▼                       ▼
   ┌───────────────┐       ┌───────────────┐
   │ 寻找最佳备用 core│       │ fail_count 递增 │
   │ coreBetterScore │       │ 0...10         │
   └───────┬───────┘       └───────┬───────┘
           │                       │
           ▼                       │
   ┌───────────────────┐           │
   │ 切换条件 (任一):    │           │
   │ • errorScore > 1.0 │           │
   │ • !healthy()       │           │
   │ • 累积误差 ≤ -0.5  │           │
   └───────┬───────────┘           │
           │                       │
           ▼                       ▼
   ┌───────────────┐       ┌───────────────┐
   │ switchLane()  │       │ count=8 yaw复位│
   │ 执行通道切换    │       │ count=9 通道切换│
   └───────────────┘       │ count=10 触发! │
                           └───────────────┘
```

---

## 九、关键参数汇总

| 参数 | 默认值 | 说明 |
|---|---|---|
| `EK3_PRIMARY` | 0 | 启动/未解锁时偏好的 core 编号 |
| `EK3_IMU_MASK` | 1 | 哪些 IMU 启用 EKF core（位掩码） |
| `EK3_ERR_THRESH` | 0.2 | 累积相对误差的最小改善阈值 |
| `EK3_AFFINITY` | 0 | 传感器到核心的亲和性位掩码（GPS/Baro/Compass/Airspeed） |
| `CORE_ERR_LIM` | 1.0 | 累积相对误差的钳位范围 `[-1.0, 1.0]` |
| `BETTER_THRESH` | 0.5 | 累积误差低于此值视为"显著更好" |
| `EK3_VEL_I_GATE` | 500 (5.0σ) | GPS 速度 innovation gate |
| `EK3_POS_I_GATE` | 500 (5.0σ) | GPS 位置 innovation gate |
| `EK3_HGT_I_GATE` | 500 (5.0σ) | 高度 innovation gate |
| `EK3_MAG_I_GATE` | 300 (3.0σ) | 磁力计 innovation gate |
| `FS_EKF_THRESH` | 0.0 (禁用) | 车辆层面 EKF failsafe 的方差阈值 |
| `FS_EKF_ACTION` | 1 (LAND) | 触发 failsafe 后的动作 |
| `EKF_CHECK_ITERATIONS_MAX` | 10 | 连续失败多少次触发 failsafe（10 = 1秒） |

---

## 十、总结

整个 EKF3 core 评价与切换体系可以概括为：

1. **每个 core 独立运行**，每时每刻都有自己的 `errorScore`（瞬时融合质量）和 `healthy`（是否可用）

2. **Primary core 是基准线**，它的 `errorScore` 直接用于绝对判断（>1.0 就切），不需要累积

3. **备用 core 要"挑战" primary**，需要长期累积证据（`coreRelativeErrors`），达到 -0.5 才能取代健康的 primary

4. **两条切换路径互补**：正常路径靠累积误差（稳定），故障路径靠瞬时分数（快速）

5. **车辆层面 failsafe 是最后防线**：通过投票机制检测 primary core 的多传感器异常，触发前还有 yaw 复位和通道切换两次补救机会
