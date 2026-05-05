# ArduPilot 罗盘校准机制详解

## 1. 概述

罗盘（磁力计）是无人机获取航向角的重要传感器之一。然而，磁力计的测量值会受到多种误差的影响，导致方向判断不准确。ArduPilot 采用**椭球拟合算法**（Ellipsoid Fitting）对罗盘进行校准，通过采集飞行器在各个方向旋转时的磁力计数据，拟合出一个数学椭球模型，并计算出校正参数，将椭球"矫正"为以原点为中心的球体。

### 1.1 校准能修正哪些误差

根据 `CompassCalibrator.cpp` 头部的注释，校准算法旨在消除以下几类误差：

| 误差类型 | 描述 | 对应校准参数 |
|---------|------|------------|
| **Sensor bias error** | 传感器本身的零偏误差 | `offset` |
| **Hard iron error** | 固定在机体上的硬磁性材料（如铁磁性金属）产生的静态干扰磁场 | `offset` |
| **Sensor scale-factor error** | 传感器比例因子误差 | `scale_factor` |
| **Sensor cross-axis sensitivity** | 传感器 cross-axis 灵敏度 | `offdiag` |
| **Soft iron error** | 固定在机体上的软磁性材料扭曲周围磁场导致的误差 | `diag` + `offdiag` |

### 1.2 核心思想

地球磁场是一个近似均匀的磁场，理想情况下，当飞行器在原地旋转一周时，罗盘测得的磁场向量端点应该分布在一个以原点为球心、半径等于当地地磁场强度的**球面**上。由于上述各种误差的存在，实际测量到的是一个**椭球面**。

校准的目标就是：找到一组仿射变换参数（offset、diag、offdiag），使得这个椭球面"还原"为球面。

校正公式为：

```
corrected_field = S × (raw_field + offset)
```

其中 **S** 是由 `diag`（对角线）和 `offdiag`（非对角线）组成的 3×3 软铁补偿矩阵。

---

## 2. 代码架构

### 2.1 核心类

- **`CompassCalibrator`**（`CompassCalibrator.h` / `.cpp`）：核心校准算法实现，负责状态机管理、样本采集、椭球拟合、方向检测等。
- **`Compass`**（`AP_Compass.h` / `.cpp`）：罗盘驱动程序，管理所有罗盘实例和校准状态。
- **`AP_Compass_Calibration.cpp`**：校准流程管理，包括 MAVLink 命令处理、启动/停止/接受校准等。

### 2.2 关键文件

```
libraries/AP_Compass/
├── CompassCalibrator.h       # 核心校准算法类定义
├── CompassCalibrator.cpp     # 核心校准算法实现（Levenberg-Marquardt拟合）
├── AP_Compass_Calibration.cpp # 校准流程管理（MAVLink命令处理）
├── AP_Compass.h              # 罗盘驱动主类
├── AP_Compass_Backend.cpp    # 罗盘后端驱动基类（调用new_sample）
└── ...
```

---

## 3. 校准参数详解

### 3.1 参数结构

校准参数存储在 `param_t` 结构体中（`CompassCalibrator.h:109-124`）：

```cpp
class param_t {
    float radius;       // 磁场强度（从样本拟合计算得出）
    Vector3f offset;    // 偏移量（校正硬铁误差和传感器零偏）
    Vector3f diag;      // 对角线缩放系数（校正软铁误差）
    Vector3f offdiag;   // 非对角线缩放系数（校正软铁误差和cross-axis灵敏度）
    float scale_factor; // 比例因子（校正传感器整体比例误差）
};
```

### 3.2 校正矩阵

软铁误差通过一个 3×3 矩阵进行校正：

```
         | diag.x    offdiag.x offdiag.y |
S  =     | offdiag.x diag.y    offdiag.z |
         | offdiag.y offdiag.z diag.z    |
```

最终的校正公式：

```
corrected = S × (raw + offset)
```

---

## 4. 状态机

校准过程由一个状态机控制（`CompassCalibrator.h:39-48`）：

```cpp
enum class Status {
    NOT_STARTED = 0,      // 未开始
    WAITING_TO_START = 1, // 等待开始（可设置延迟启动）
    RUNNING_STEP_ONE = 2, // 第一阶段：采集样本 + 球面拟合
    RUNNING_STEP_TWO = 3, // 第二阶段：采集更多样本 + 椭球拟合
    SUCCESS = 4,          // 校准成功
    FAILED = 5,           // 校准失败
    BAD_ORIENTATION = 6,  // 罗盘方向错误
    BAD_RADIUS = 7,       // 磁场半径异常（与预期不符）
};
```

### 4.1 状态转换图

```
                         ┌─────────────────┐
                         │  NOT_STARTED    │
                         └────────┬────────┘
                                  │ start() 调用
                                  ▼
                         ┌─────────────────┐
                         │ WAITING_TO_START │ ←── delay 参数控制
                         └────────┬────────┘
                                  │ 延迟结束 + 内存分配成功
                                  ▼
                         ┌─────────────────┐
                         │ RUNNING_STEP_ONE │ ← 采集300个样本
                         │  run_sphere_fit()│    球面拟合（4参数）
                         └────────┬────────┘
                                  │ 10步迭代完成
                                  ▼
                         ┌─────────────────┐
                         │ RUNNING_STEP_TWO │ ← 稀疏样本后再采集到300个
                         │ run_ellipsoid_fit│    椭球拟合（9参数）
                         └────────┬────────┘
                                  │ 35步迭代完成
                                  ▼
              ┌────────────────────┼────────────────────┐
              ▼                    ▼                    ▼
       ┌──────────┐          ┌──────────┐         ┌──────────┐
       │ SUCCESS  │          │  FAILED  │         │BAD_RADIUS│
       └──────────┘          └──────────┘         └──────────┘
              │                    │                    │
              │ retry=true         │ retry=true         │ retry=true
              └────────────────────┴────────────────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │ WAITING_TO_START │ (attempt++)
                         └─────────────────┘
```

---

## 5. 核心算法详解

### 5.1 样本采集

罗盘后端驱动通过 `publish_raw_field()` 函数（`AP_Compass_Backend.cpp:59-72`）不断将原始磁力计数据送入校准器：

```cpp
void AP_Compass_Backend::publish_raw_field(const Vector3f &mag)
{
    auto& cal = _compass._calibrator[...];
    if (cal != nullptr) {
        cal->new_sample(mag);  // 将样本送入校准器
    }
}
```

在 `CompassCalibrator::new_sample()`（`CompassCalibrator.cpp:129-135`）中，不仅记录磁力计数据，还同时记录当前的姿态：

```cpp
void CompassCalibrator::new_sample(const Vector3f& sample)
{
    WITH_SEMAPHORE(sample_sem);
    _last_sample.set(sample);
    _last_sample.att.set_from_ahrs();  // 从AHRS获取roll/pitch/yaw
    _new_sample = true;
}
```

这样做的目的是：后续可以验证在不同的飞行器姿态下，同一地球磁场向量经过旋转后是否一致，从而检测罗盘的安装方向是否正确。

### 5.2 样本接受标准

为了保证拟合质量，样本必须在空间上**均匀分布**在球面上。算法基于球面几何计算最小接受距离（`CompassCalibrator.cpp:542-562`）：

```cpp
bool CompassCalibrator::accept_sample(const Vector3f& sample, uint16_t skip_index)
{
    // 基于正多面体几何计算最小接受距离
    static const float theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));
    float min_distance = _params.radius * 2 * sinf(theta / 2);

    // 检查与所有已有样本的距离
    for (uint16_t i = 0; i < _samples_collected; i++) {
        float distance = (sample - _sample_buffer[i].get()).length();
        if (distance < min_distance) {
            return false;  // 太近，拒绝
        }
    }
    return true;
}
```

这个设计确保：
1. 样本不会在空间中聚集
2. 样本覆盖球面的各个方向
3. 总样本数固定为 300 个

### 5.3 Levenberg-Marquardt 拟合算法

ArduPilot 采用 **Levenberg-Marquardt（LM）算法** 进行非线性最小二乘拟合。LM 算法是 Gauss-Newton 算法和梯度下降法的混合，既具有 Gauss-Newton 的快速收敛性，又通过阻尼参数 λ 保证了数值稳定性。

#### 5.3.1 第一阶段：球面拟合（Step One）

球面方程：`radius² = |S × (sample + offset)|²`

待拟合参数为 4 个：`radius` 和 `offset`（3个分量）。

```cpp
void CompassCalibrator::run_sphere_fit()
{
    // 1. 计算雅可比矩阵 JTJ 和 JTFI
    for (uint16_t k = 0; k < _samples_collected; k++) {
        calc_sphere_jacob(sample, fit1_params, sphere_jacob);
        for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_SPHERE_PARAMS; j++) {
                JTJ[i*4+j] += sphere_jacob[i] * sphere_jacob[j];
            }
            JTFI[i] += sphere_jacob[i] * calc_residual(sample, fit1_params);
        }
    }

    // 2. Levenberg-Marquardt 阻尼
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
        JTJ[i*4+i] += _sphere_lambda;
    }

    // 3. 矩阵求逆
    mat_inverse(JTJ, JTJ, 4);

    // 4. 更新参数
    for (uint8_t row = 0; row < COMPASS_CAL_NUM_SPHERE_PARAMS; row++) {
        for (uint8_t col = 0; col < COMPASS_CAL_NUM_SPHERE_PARAMS; col++) {
            fit1_params.get_sphere_params()[row] -= JTFI[col] * JTJ[row*4+col];
        }
    }

    // 5. 评估新的 fitness，如果变差则调整 lambda
    fit1 = calc_mean_squared_residuals(fit1_params);
    if (fit1 > _fitness && fit2 > _fitness) {
        _sphere_lambda *= lma_damping;  // 增加阻尼，接近梯度下降
    } else if (fit2 < _fitness && fit2 < fit1) {
        _sphere_lambda /= lma_damping;  // 减少阻尼，接近 Gauss-Newton
        fit1_params = fit2_params;
    }
}
```

球面拟合进行 10 次迭代。

#### 5.3.2 第二阶段：椭球拟合（Step Two）

椭球拟合参数为 9 个：3 个 `offset` + 3 个 `diag` + 3 个 `offdiag`。

```cpp
void CompassCalibrator::run_ellipsoid_fit()
{
    // 类似于球面拟合，但参数数量为 9
    for (uint16_t k = 0; k < _samples_collected; k++) {
        calc_ellipsoid_jacob(sample, fit1_params, ellipsoid_jacob);
        // ... 计算 JTJ (9x9) 和 JTFI (9)
    }

    // LM 阻尼
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
        JTJ[i*9+i] += _ellipsoid_lambda;
    }

    mat_inverse(JTJ, JTJ, 9);
    // ... 更新 9 个参数
}
```

椭球拟合进行 35 次迭代（其中前 15 次继续做球面拟合，后 20 次做椭球拟合）。

### 5.4 拟合接受标准

不是任何拟合结果都会被接受。`fit_acceptable()` 函数（`CompassCalibrator.cpp:480-496`）检查：

```cpp
bool CompassCalibrator::fit_acceptable() const
{
    if (isnan(_fitness) &&  // fitness 不能是 NaN
        _params.radius > FIELD_RADIUS_MIN && _params.radius < FIELD_RADIUS_MAX &&  // 半径在合理范围
        fabsf(_params.offset.x) < _offset_max &&  // offset 不能过大
        fabsf(_params.offset.y) < _offset_max &&
        fabsf(_params.offset.z) < _offset_max &&
        _params.diag.x > 0.2f && _params.diag.x < 5.0f &&  // diag 在合理范围
        _params.diag.y > 0.2f && _params.diag.y < 5.0f &&
        _params.diag.z > 0.2f && _params.diag.z < 5.0f &&
        fabsf(_params.offdiag.x) < 1.0f &&  // offdiag 不能过大
        fabsf(_params.offdiag.y) < 1.0f &&
        fabsf(_params.offdiag.z) < 1.0f) {
        return _fitness <= sq(_tolerance);  // 均方残差小于阈值
    }
    return false;
}
```

### 5.5 方向自动检测

`calculate_orientation()` 函数（`CompassCalibrator.cpp:920-1070`）可以自动检测罗盘的安装方向是否正确：

1. 尝试所有可能的旋转方向（共 ROTATION_MAX - 4 种）
2. 对于每种方向，计算所有样本转换到地球坐标系后的**方差**
3. 方差最小的方向就是"最正确"的方向
4. 如果检测到的方向与预期方向不一致，会根据设置自动修正或报告错误

```cpp
Vector3f CompassCalibrator::calculate_earth_field(CompassSample &sample, enum Rotation r)
{
    Vector3f v = sample.get();

    // 转换回传感器坐标系
    v.rotate_inverse(_orientation);
    // 旋转到待测试的机体坐标系
    v.rotate(r);

    // 应用偏移
    v += rot_offsets;

    // 旋转回地球坐标系
    Matrix3f rot = sample.att.get_rotmat();
    Vector3f efield = rot * v;

    return efield;
}
```

### 5.6 半径修正

`fix_radius()` 函数（`CompassCalibrator.cpp:1076-1108`）根据 GPS 位置和 WMM（世界磁场模型）表格，对拟合半径进行修正：

```cpp
bool CompassCalibrator::fix_radius()
{
    // 获取当前位置
    Location loc;
    AP::ahrs().get_location(loc);

    // 查询该位置的理论磁场强度
    float intensity, declination, inclination;
    AP_Declination::get_mag_field_ef(loc.lat * 1e-7f, loc.lng * 1e-7f,
                                      intensity, declination, inclination);

    float expected_radius = intensity * 1000; // mGauss
    float correction = expected_radius / _params.radius;

    // 限制修正范围（0.7 ~ 1.3）
    if (correction > COMPASS_MAX_SCALE_FACTOR || correction < COMPASS_MIN_SCALE_FACTOR) {
        set_status(Status::BAD_RADIUS);  // 修正过大，说明校准有问题
        return false;
    }

    _params.scale_factor = correction;
    return true;
}
```

---

## 6. 校准启动与流程管理

### 6.1 MAVLink 命令入口

校准由地面站通过 MAVLink 命令 `MAV_CMD_DO_START_MAG_CAL` 触发（`AP_Compass_Calibration.cpp:381-410`）：

```cpp
case MAV_CMD_DO_START_MAG_CAL: {
    uint8_t mag_mask = packet.param1;  // 罗盘掩码（0表示所有罗盘）
    bool retry = !is_zero(packet.param2);      // 失败后是否自动重试
    bool autosave = !is_zero(packet.param3);    // 成功后是否自动保存
    float delay = packet.param4;                 // 启动延迟（秒）
    bool autoreboot = packet.x != 0;            // 完成后是否自动重启

    if (mag_mask == 0) {
        start_calibration_all(retry, autosave, delay, autoreboot);
    } else {
        _start_calibration_mask(mag_mask, retry, autosave, delay, autoreboot);
    }
}
```

### 6.2 校准线程

校准算法运行在一个独立的低优先级线程中（`AP_Compass_Calibration.cpp:98-104`）：

```cpp
if (!_cal_thread_started) {
    hal.scheduler->thread_create(
        FUNCTOR_BIND(this, &Compass::_update_calibration_trampoline, void),
        "compasscal", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0
    );
    _cal_thread_started = true;
}
```

线程主循环（`AP_Compass_Calibration.cpp:113-123`）：

```cpp
void Compass::_update_calibration_trampoline() {
    while(true) {
        for (uint8_t i = 0; i < COMPASS_MAX_INSTANCES; i++) {
            if (_calibrator[i] != nullptr) {
                _calibrator[i]->update();  // 更新每个罗盘的校准状态
            }
        }
        hal.scheduler->delay(1);  // 1ms 周期
    }
}
```

### 6.3 校准更新主循环

`CompassCalibrator::update()`（`CompassCalibrator.cpp:175-232`）是校准的主更新函数：

```cpp
void CompassCalibrator::update()
{
    // 1. 从中间结构取出样本
    pull_sample();

    // 2. 检查是否开始拟合（样本数达到300）
    if (!_fitting()) {
        return;
    }

    // 3. 第一阶段：球面拟合
    if (_status == Status::RUNNING_STEP_ONE) {
        if (_fit_step >= 10) {
            if (is_equal(_fitness, _initial_fitness) || isnan(_fitness)) {
                set_status(Status::FAILED);  // 拟合发散
            } else {
                set_status(Status::RUNNING_STEP_TWO);
            }
        } else {
            if (_fit_step == 0) {
                calc_initial_offset();  // 初始偏移 = 样本均值
            }
            run_sphere_fit();
            _fit_step++;
        }
    }
    // 4. 第二阶段：椭球拟合
    else if (_status == Status::RUNNING_STEP_TWO) {
        if (_fit_step >= 35) {
            if (fit_acceptable() && fix_radius() && calculate_orientation()) {
                set_status(Status::SUCCESS);
            } else {
                set_status(Status::FAILED);
            }
        } else if (_fit_step < 15) {
            run_sphere_fit();
            _fit_step++;
        } else {
            run_ellipsoid_fit();
            _fit_step++;
        }
    }
}
```

---

## 7. 校准参数保存

校准成功后，参数通过以下函数保存到持久化存储（`AP_Compass_Calibration.cpp:187-223`）：

```cpp
bool Compass::_accept_calibration(uint8_t i)
{
    const CompassCalibrator::Report cal_report = cal->get_report();

    if (cal_report.status == CompassCalibrator::Status::SUCCESS) {
        Vector3f ofs(cal_report.ofs), diag(cal_report.diag), offdiag(cal_report.offdiag);
        float scale_factor = cal_report.scale_factor;

        // 保存偏移量
        set_and_save_offsets(i, ofs);

        // 保存对角线和非对角线缩放
        set_and_save_diagonals(i, diag);
        set_and_save_offdiagonals(i, offdiag);

        // 保存比例因子
        set_and_save_scale_factor(i, scale_factor);

        // 如果方向被修正，保存新方向
        if (cal_report.check_orientation && _get_state(prio).external && _rotate_auto >= 2) {
            set_and_save_orientation(i, cal_report.orientation);
        }

        AP_Notify::events.compass_cal_saved = 1;
    }
}
```

---

## 8. 物理意义总结

| 参数 | 物理意义 | 如何获得 |
|------|---------|---------|
| `offset` | 硬铁磁场 + 传感器零偏，在所有方向上产生相同偏移 | 样本的算术平均值（取负） |
| `diag` | 传感器在三个轴向上的不同灵敏度 | 椭球拟合 |
| `offdiag` | 传感器轴间的非正交性 + 软铁效应 | 椭球拟合 |
| `radius` | 当地地球磁场强度 | 椭球拟合 |
| `scale_factor` | 传感器整体比例误差 | 通过 GPS 位置查 WMM 表格获得理论磁场强度，与 `radius` 比较得到 |

---

## 9. 实际操作流程

1. **地面站发送** `MAV_CMD_DO_START_MAG_CAL` 命令（必须处于未解锁状态）
2. **飞控启动校准**，地面站显示进度百分比
3. **用户手动旋转飞行器**，在各个方向上采集磁力计样本
4. **校准完成后**，地面站显示结果（SUCCESS / FAILED）
5. 如果设置了 `autosave=true`，参数自动保存；否则需要发送 `MAV_CMD_DO_ACCEPT_MAG_CAL` 确认保存

---

## 10. 关键参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `COMPASS_CAL_NUM_SAMPLES` | 300 | 每个阶段采集的样本数量 |
| `COMPASS_CAL_NUM_SPHERE_PARAMS` | 4 | 球面拟合参数数量 |
| `COMPASS_CAL_NUM_ELLIPSOID_PARAMS` | 9 | 椭球拟合参数数量 |
| `FIELD_RADIUS_MIN` | 150 | 最小可接受磁场强度（mGauss） |
| `FIELD_RADIUS_MAX` | 950 | 最大可接受磁场强度（mGauss） |
| `lma_damping` | 10.0 | LM 算法阻尼因子 |

---

## 11. 参考

- Levenberg-Marquardt 算法：https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
- ArduPilot 官方文档：https://ardupilot.org/copter/docs/common-compass-calibration.html
