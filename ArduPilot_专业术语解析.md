# ArduPilot 无人机自动驾驶仪专业术语解析

> 本文档为 ArduPilot 开源无人机自动驾驶仪项目的专业术语解析，适合初学者入门学习。

---

## 一、项目基础概念

### 1.1 什么是 ArduPilot

**ArduPilot** 是一个功能强大、可靠的开源自动驾驶仪软件系统。该项目始于2010年，由来自不同背景的专业工程师、计算机科学家和社区贡献者共同开发。它是世界上最先进、最成熟的开源无人机自驾仪系统。

- **官方网站**: https://ardupilot.org
- **GitHub**: https://github.com/ArduPilot/ardupilot
- **许可证**: GNU General Public License v3 (GPLv3)

### 1.2 ArduPilot 支持的车辆类型

| 名称 | 代码目录 | 说明 |
|------|---------|------|
| **ArduCopter** | `ArduCopter/` | 多旋翼无人机（直升机、四轴、六轴、八轴等）自驾仪 |
| **ArduPlane** | `ArduPlane/` | 固定翼飞机自驾仪，支持常规飞机、VTOL、倾转旋翼机等 |
| **Rover** | `Rover/` | 地面车辆（漫游车、机器人）自驾仪 |
| **ArduSub** | `ArduSub/` | 水下航行器（潜艇/ROV）自驾仪 |
| **AntennaTracker** | `AntennaTracker/` | 天线追踪器 |
| **Blimp** | `Blimp/` | 飞艇自驾仪 |

---

## 二、硬件相关术语

### 2.1 飞行控制器 (Flight Controller / FC)

**飞行控制器**是无人机的"大脑"，负责处理传感器数据、运行控制算法并输出电机控制信号。

**常见硬件平台**:
- **Pixhawk 系列**: 最流行的开源飞控硬件平台
- **CubeOrange / CubeBlack**: Pixhawk 的高端版本
- **Durandal**: Holybro 生产的飞控
- **KakuteF7 / F4**: 对角线设计的飞控
- **Holybro Pixhawk 系列**: 包括 Pixhawk 4, Pixhawk 5, Pixhawk 6

### 2.2 HAL (Hardware Abstraction Layer)

**硬件抽象层**，是 ArduPilot 代码架构中的一层，提供了统一的接口来访问不同硬件平台的外设。

```cpp
// 示例：HAL 接口示例
AP_HAL::HAL* hal = AP_HAL::get_HAL();
hal->gpio->pinMode(LED_PIN, OUTPUT);
```

**常见的 HAL 实现**:
- **AP_HAL_ChibiOS**: 基于 ChibiOS RTOS 的 HAL，用于大多数 ARM Cortex-M 飞控
- **AP_HAL_Linux**: 用于 Linux SBC (如 BeagleBone Blue, NavIO)
- **AP_HAL_ESP32**: 用于 ESP32 微控制器
- **AP_HAL_SITL**: 软件在环仿真 (Simulation In The Loop)

### 2.3 传感器相关

#### IMU (Inertial Measurement Unit) - 惯性测量单元

IMU 是无人机的核心传感器，包含：
- **加速度计 (Accelerometer)**: 测量三个轴向的加速度
- **陀螺仪 (Gyroscope)**: 测量三个轴向的角速度

#### GPS (Global Positioning System) - 全球定位系统

用于获取无人机的位置、速度和时间信息。

- **GNSS**: 全球导航卫星系统（GPS, GLONASS, Galileo, BeiDou 等）

#### 磁力计 / 罗盘 (Magnetometer / Compass)

用于确定无人机的航向（相对于地球磁北的方向）。

#### 气压计 (Barometer / Baro)

测量大气压力，用于估算飞行高度。

#### 超声波测距仪 (Sonar / Range Finder)

用于测量到地面的距离，实现低空飞行和避障。

---

## 三、核心算法与控制

### 3.1 PID 控制

**PID (Proportional-Integral-Derivative) 控制器**是自动控制领域最经典的算法，用于维持期望的状态。

```
输出 = Kp × 误差 + Ki × 误差积分 + Kd × 误差变化率
```

- **P (比例)**: 误差越大，纠正力度越大
- **I (积分)**: 消除长期累积的误差
- **D (微分)**: 预测误差趋势，提前纠正

在 ArduPilot 中，PID 控制用于：
- 姿态控制 (Roll, Pitch, Yaw)
- 位置控制 (X, Y, Z)
- 速度控制
- 高度控制

### 3.2 EKF (Extended Kalman Filter) - 扩展卡尔曼滤波

**卡尔曼滤波**是一种递归滤波器，用于从带噪声的传感器数据中估计系统状态。

**ArduPilot 中的 EKF**:
- **AP_NavEKF2**: 第二代导航扩展卡尔曼滤波器
- **AP_NavEKF3**: 第三代导航扩展卡尔曼滤波器（更先进）

EKF 融合多种传感器数据：
- GPS 位置/速度
- IMU 加速度/角速度
- 磁力计航向
- 气压计高度
- 超声波测距

### 3.3 姿态 (Attitude) 与姿态控制

**姿态**指无人机在三维空间中的方向，由三个角度描述：
- **Roll (横滚)**: 绕前后轴的旋转
- **Pitch (俯仰)**: 绕左右轴的旋转
- **Yaw (偏航)**: 绕垂直轴的旋转（航向）

```
     Pitch (俯仰)
         ↑
         |
         |
    ←────┼────→ Roll (横滚)
         |
         |
         ↓
    Yaw (偏航)
```

### 3.4 飞行模式 (Flight Mode)

ArduPilot 支持多种飞行模式：

| 模式 | 说明 | 适用场景 |
|------|------|---------|
| **Stabilize** | 稳定模式，遥控器控制姿态，飞行器自稳 | 新手学习 |
| **Alt Hold** | 高度保持模式 | 航拍 |
| **Loiter** | 定点悬停模式 | 定点飞行 |
| **RTL (Return to Launch)** | 返航模式 | 紧急返航 |
| **Land** | 降落模式 | 自动降落 |
| **Auto** | 自动模式，按预设航线飞行 | 测绘、巡检 |
| **Guided** | 引导模式，由地面站或API控制 | 高级应用 |
| **Acro** | 特技模式，无自稳 | 花式飞行 |

---

## 四、通信协议

### 4.1 MAVLink (Micro Air Vehicle Link)

**MAVLink** 是无人机领域最广泛使用的通信协议，用于地面站与飞行器之间的通信。

**特点**:
- 轻量级、高效率
- 支持多种消息类型（遥测、指令、参数等）
- 跨平台（地面站软件如 Mission Planner, QGroundControl）

**常见消息**:
- `HEARTBEAT`: 心跳消息
- `MISSION_ITEM`: 航线航点
- `GLOBAL_POSITION`: 全球位置
- `ATTITUDE`: 姿态数据
- `RC_CHANNELS`: 遥控器通道

### 4.2 CAN 总线与 DroneCAN

**CAN (Controller Area Network)** 是一种可靠的车辆网络协议。

**DroneCAN** (原 UAVCAN) 是专门为无人机设计的 CAN 协议，用于：
- 电机ESC与飞控通信
- 传感器数据传输
- GPS/罗盘等外设连接

### 4.3 串口通信

- **UART**: 通用异步收发传输协议，用于连接 GPS、数传模块等
- **I2C**: 用于连接罗盘、气压计等 I2C 设备
- **SPI**: 高速通信，用于连接 IMU 传感器

---

## 五、导航与规划

### 5.1 地理围栏 (Geo-fence)

**地理围栏**是虚拟的边界，用于限制无人机的飞行区域。当无人机接近或超出围栏时，会自动执行返航或降落。

### 5.2 航线 (Mission)

**航线**是预先规划的一系列航点（Waypoint），无人机将按顺序飞向每个航点。

**常见的航线动作**:
- **WAYPOINT**: 飞向指定位置
- **TAKEOFF**: 起飞到指定高度
- **LAND**: 降落
- **RTL**: 返航
- **DO_CHANGE_SPEED**: 改变速度
- **DO_SET_ROI**: 设置兴趣点（相机朝向）

### 5.3 航点 (Waypoint)

航点是航线中的具体位置点，包含：
- 经度 (Longitude)
- 纬度 (Latitude)
- 高度 (Altitude)
- 到达后等待时间

### 5.4 ROI (Region of Interest) - 兴趣点

指定无人机相机或云台应该朝向的目标位置，常用于航拍时跟随地面目标。

---

## 六、动力与电池

### 6.1 ESC (Electronic Speed Controller) - 电子调速器

ESC 负责将飞控的PWM信号转换为驱动无刷电机所需的电流。

**常见协议**:
- **PWM ESC**: 传统 PWM 信号
- **OneShot125**: 更快响应速度的 PWM 协议
- **DShot**: 数字协议，更高精度和响应速度
- **BiDirectional DShot**: 支持双向通信，可回传 ESC 状态

### 6.2 电池相关

- **S 数 (S Number)**: 锂电池串联数量（3S = 3节串联 = 11.1V）
- **C 数 (C Rating)**: 电池放电能力倍率
- **mAh**: 电池容量单位
- **电压阈值**: 低压保护阈值（通常每个电芯 3.3V - 3.0V）

---

## 七、开发与测试

### 7.1 SITL (Software In The Loop)

**软件在环仿真**，是在计算机上运行的完整飞行模拟，无需实际硬件即可测试飞行代码。

**优点**:
- 无需硬件即可开发和测试
- 可以模拟各种飞行场景
- 安全的测试环境

```bash
# 启动 SITL 示例
cd ardupilot/ArduCopter
sim_vehicle.py --map --console
```

### 7.2 HIL (Hardware In The Loop)

**硬件在环仿真**，使用真实飞控硬件与仿真软件连接，测试飞控在真实硬件上的表现。

### 7.3 MVProxy 是功能强大的 MAVLinkAVProxy

MA 地面站软件，可用于：
- 无人机遥测数据监控
- 航线规划
- 飞行模拟
- 模块化扩展

### 7.4 Ground Control Station (GCS) - 地面站

用于监控和控制无人机的地面软件：

- **Mission Planner**: 最流行的 ArduPilot 地面站（仅 Windows）
- **QGroundControl**: 跨平台地面站
- **APM Planner 2**: 跨平台替代方案

### 7.5 Waf 构建系统

ArduPilot 使用 **Waf** 作为构建系统（替代传统的 Makefile）。

**常用命令**:
```bash
# 配置构建环境
./waf configure --board sitl        # 配置 SITL 仿真
./waf configure --board CubeBlack  # 配置特定硬件

# 构建车辆
./waf copter    # 构建多旋翼
./waf plane     # 构建固定翼
./waf rover     # 构建车辆
./waf sub       # 构建水下航行器

# 列出支持的硬件
./waf list_boards

# 清理构建
./waf clean        # 清理当前目标
./waf distclean    # 清理所有
```

### 7.6 GTest 单元测试

ArduPilot 使用 **Google Test (GTest)** 框架进行 C++ 单元测试。

**测试文件位置**: `libraries/<库名>/tests/`

```cpp
#include <AP_gtest.h>
TEST(MathTest, IsZero) {
    EXPECT_TRUE(is_zero(0.0f));
    EXPECT_FALSE(is_zero(1.0f));
}
```

### 7.7 Autotest 集成测试

**Autotest** 是 ArduPilot 的自动化集成测试框架，位于 `Tools/autotest/` 目录。

**运行测试**:
```bash
# 运行特定测试
Tools/autotest/autotest.py build.Copter test.Copter.RTLYaw

# 运行车辆所有测试
Tools/autotest/autotest.py build.Copter test.Copter
```

### 7.8 CI (持续集成)

ArduPilot 使用 GitHub Actions 进行持续集成，PR 提交时会自动运行：
- SITL 测试
- C++ 单元测试
- 代码格式化检查 (astyle)
- Python 代码检查 (flake8)
- 提交信息格式检查

### 7.9 代码格式化工具

- **astyle**: C++ 代码格式化工具，确保代码风格一致
- **flake8**: Python 代码检查工具
- **Pre-commit hooks**: 提交前自动检查工具

### 7.10 硬件调试

#### Black Magic Probe

**Black Magic Probe** 是一款开源的 ARM JTAG 调试器，可用于调试飞控硬件。

#### STLink-v2

**STLink-v2** 是 ST 公司生产的调试器，配合 OpenOCD 使用。

#### OpenOCD

**OpenOCD** (Open On-Chip Debugger) 是开源的调试软件，支持多种调试器。

#### GDB

**GDB** (GNU Debugger) 是 GNU 项目的调试器，用于调试飞控程序。

#### JTAG / SWD

- **JTAG**: 标准的调试接口
- **SWD** (Serial Wire Debug): ARM 专用的两线调试接口

### 7.11 ChibiOS 实时操作系统

**ChibiOS** 是 ArduPilot 主要使用的实时操作系统 (RTOS)，运行在 ARM Cortex-M 微控制器上。

---

## 八、高级功能

### 8.1 VTOL (Vertical Take-Off and Landing) - 垂直起降

**VTOL** 是结合固定翼和多旋翼优点的飞行器：
- 垂直起降（像多旋翼）
- 水平飞行（像固定翼）

**常见类型**:
- **QuadPlane**: 四旋翼 + 固定翼
- **Tilt-rotor**: 倾转旋翼（旋翼可倾转）

### 8.2 避障 (Obstacle Avoidance)

ArduPilot 支持多种避障方案：
- **光流 (Optical Flow)**: 基于视觉的室内定位和避障
- **测距仪 (Range Finder)**: 超声波/激光测距
- **视觉避障 (Vision Avoidance)**: 基于摄像头的障碍物检测

### 8.3 精确着陆 (Precision Landing)

使用视觉或信标系统实现精确降落，例如：
- **PX4FLOW**: 光流传感器
- **RTK GPS**: 厘米级定位精度

### 8.4 ROS2 集成

ArduPilot 支持与 **ROS2 (Robot Operating System 2)** 集成，用于：
- 机器人导航
- 自主避障
- 人工智能应用

#### ROS2 相关术语

- **colcon**: ROS2 的构建工具，用于编译 ROS2 包
- **micro-ROS**: 专为微控制器设计的 ROS2 实现
- **DDS** (Data Distribution Service): 数据分发服务，ROS2 的通信中间件
- **launch files**: ROS2 启动文件，用于配置和启动节点
- **AP_DDS**: ArduPilot 的 DDS 客户端库

```bash
# ROS2 启动 SITL 示例
ros2 launch ardupilot_sitl sitl.launch.py command:=ardurover model:=rover

# 启动带 MAVProxy
ros2 launch ardupilot_sitl sitl_mavproxy.launch.py map:=True console:=True
```

---

## 九、代码架构

### 9.1 目录结构

```
ardupilot/
├── ArduCopter/      # 多旋翼飞控代码
├── ArduPlane/      # 固定翼飞控代码
├── Rover/          # 车辆飞控代码
├── ArduSub/        # 水下航行器代码
├── AntennaTracker/ # 天线追踪器代码
├── libraries/      # 共享库
│   ├── AC_AttitudeControl/  # 姿态控制
│   ├── AP_NavEKF2/         # EKF2 滤波器
│   ├── AP_NavEKF3/         # EKF3 滤波器
│   ├── AP_GPS/             # GPS 驱动
│   ├── AP_Baro/            # 气压计驱动
│   ├── AP_Compass/         # 罗盘驱动
│   ├── AP_Motors/          # 电机控制
│   ├── AP_Mission/         # 任务规划
│   ├── GCS_MAVLink/        # MAVLink 通信
│   └── AP_HAL_ChibiOS/    # 硬件抽象层
└── Tools/          # 工具和脚本
```

### 9.2 主要库模块

| 库名称 | 功能 |
|--------|------|
| **AC_AttitudeControl** | 姿态控制算法 |
| **AC_PID** | PID 控制器实现 |
| **AC_AutoTune** | 自动调参功能 |
| **AC_Avoidance** | 避障功能 |
| **AC_Fence** | 地理围栏功能 |
| **AC_PrecLand** | 精确着陆功能 |
| **AC_WPNav** | 航点导航 |
| **AP_NavEKF2/EKF3** | 导航状态估计 |
| **AP_GPS** | GPS 接收机驱动 |
| **AP_Baro** | 气压计驱动 |
| **AP_Compass** | 电子罗盘/磁力计 |
| **AP_RangeFinder** | 测距传感器 |
| **AP_Motors** | 电机输出管理 |
| **AP_BattMonitor** | 电池监控 |
| **AP_Mission** | 任务/航线管理 |
| **AP_Fence** | 地理围栏 |
| **AP_Camera / AP_Mount** | 相机和云台控制 |
| **AP_Notify** | 通知系统（LED、蜂鸣器） |
| **AP_AHRS** | 姿态航向参考系统 |
| **AP_Arming** | 起飞前安全检查系统 |
| **AP_BLHeli** | BLHeli ESC 固件支持 |

### 9.3 代码架构模式

#### Singleton Pattern (单例模式)

ArduPilot 使用单例模式确保某些类只有一个实例：

```cpp
// 获取单例实例
AP_GPS* gps = AP_GPS::get_singleton();
```

#### Backend Interface (后端接口)

ArduPilot 的库通常采用前端-后端架构：
- **Frontend (前端)**: 主类，提供统一 API
- **Backend (后端)**: 具体驱动实现

例如：`AP_GPS` 是前端，`AP_GPS_UBLOX`、`AP_GPS_MTK` 是后端。

#### Driver Implementations (驱动实现)

每个传感器通常有多个驱动实现，支持不同厂商的硬件。

### 9.4 控制相关高级术语

#### Control Allocation (控制分配)

**控制分配**是将期望的力/力矩分配到各个执行器（电机）的过程。

#### Mixer (混控器)

**混控器**是将多个控制输入（Roll, Pitch, Yaw, Throttle）混合成各个电机输出的模块。

#### Input Shaping (输入整形)

**输入整形**是对控制输入进行滤波和平滑处理的算法，减少抖动和突变。

#### Bumpless Transfer (无扰切换)

**无扰切换**是在不同控制器之间切换时，确保输出平滑过渡的技术，避免飞行姿态突变。

#### Rate Controller (速率控制器)

**速率控制器**直接控制角速度（deg/s），而非角度，是更底层的控制环。

#### Attitude Controller (姿态控制器)

**姿态控制器**控制飞行器的角度（Roll, Pitch），通常包含输入整形和速率控制器。

---

## 十、常见配置参数

### 10.1 重要飞行参数

| 参数 | 说明 |
|------|------|
| **ARMING_CHECK** | 起飞前检查项 |
| **BRD_RADIODEBUG** | 无线调试开关 |
| **EK2_ / EK3_** | EKF2/EKF3 滤波器参数 |
| **MOT_SPIN_MIN** | 电机最小油门阈值 |
| **RC_SPEED** | PWM 输出频率 |
| **FS_THR_ENABLE** | 失控保护设置 |
| **FENCE_ENABLE** | 地理围栏开关 |

### 10.2 校准参数

- **ACCEL calibraton**: 加速度计校准
- **COMPASS calibraton**: 罗盘校准
- **Radio calibraton**: 遥控器校准
- **ESC calibraton**: 电子调速器校准

---

## 十一、故障排查术语

| 术语 | 说明 |
|------|------|
| **EKF variance** | EKF 状态估计方差过大，可能传感器有问题 |
| **GPS glitch** | GPS 信号跳变，定位不稳定 |
| **Compass interference** | 罗盘受到电磁干扰 |
| **Motor desync** | 电机不同步，可能 ESC 校准问题 |
| **Brownout** | 电压骤降导致飞控重启 |
| **Flight mode lost** | 丢失飞行模式，可能是遥控器信号问题 |

---

## 十二、学习资源

### 官方文档
- ArduPilot 官网: https://ardupilot.org
- 开发文档: https://ardupilot.org/dev/
- Copter 文档: https://ardupilot.org/copter/
- Plane 文档: https://ardupilot.org/plane/

### 社区
- 讨论论坛: https://discuss.ardupilot.org/
- Discord: https://discord.com/channels/ardupilot

### 入门建议
1. **先看文档**: 从官网 wiki 开始，学习基础概念
2. **SITL 仿真**: 使用仿真环境进行练习
3. **从简单开始**: 先掌握 Stabilize 和 Alt Hold 模式
4. **查看日志**: 使用飞行日志分析飞行数据
5. **参与社区**: 在论坛提问和学习他人经验

---

## 十二、Git 协作与代码贡献

### 12.1 基本概念

- **Fork (派生)**: 在 GitHub 上创建项目的个人副本
- **Branch (分支)**: 独立开发的工作分支
- **Master/Main**: 主分支，通常是稳定版本
- **Rebase (变基)**: 重新整理提交历史
- **Merge (合并)**: 将一个分支的更改合并到另一个分支
- **Pull Request (PR)**: 向主仓库提交代码更改的请求

### 12.2 提交信息格式

ArduPilot 要求提交信息遵循特定格式：

```
子系统: 简短描述

可选的详细说明，解释修改的动机、内容和原因。
```

**示例**:
```
AP_Terrain: add configurable cache size parameter
Copter: fix altitude hold in guided mode
Tools: improve autotest terrain data handling
```

### 12.3 参数注解

ArduPilot 使用 C++ 注解来文档化参数：

```cpp
// @Param: ENABLE
// @DisplayName: Terrain data enable
// @Description: enable terrain data. This enables the vehicle storing
//   a database of terrain data on the SD card.
// @Values: 0:Disable,1:Enable
// @User: Advanced
AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Terrain, enable, 1, AP_PARAM_FLAG_ENABLE),
```

**常用注解**:
- `@Param:`: 参数短名称
- `@DisplayName:`: 显示名称
- `@Description:`: 详细描述
- `@Values:`: 可选值列表
- `@Bitmask:`: 位掩码选项
- `@Range:`: 数值范围
- `@Units:`: 单位
- `@User:`: 用户级别 (Standard/Advanced)
- `@RebootRequired:`: 是否需要重启

---

## 附录：缩写对照表

| 缩写 | 全称 | 中文 |
|------|------|------|
| FC | Flight Controller | 飞行控制器 |
| PID | Proportional-Integral-Derivative | 比例-积分-微分 |
| EKF | Extended Kalman Filter | 扩展卡尔曼滤波 |
| IMU | Inertial Measurement Unit | 惯性测量单元 |
| GPS | Global Positioning System | 全球定位系统 |
| MAVLink | Micro Air Vehicle Link | 微小无人机通信协议 |
| ESC | Electronic Speed Controller | 电子调速器 |
| VTOL | Vertical Take-Off and Landing | 垂直起降 |
| PWM | Pulse Width Modulation | 脉宽调制 |
| CAN | Controller Area Network | 控制器局域网 |
| ROI | Region of Interest | 兴趣点 |
| RTL | Return to Launch | 返航 |
| SITL | Software In The Loop | 软件在环仿真 |
| HIL | Hardware In The Loop | 硬件在环仿真 |
| GCS | Ground Control Station | 地面站 |
| AHRS | Attitude and Heading Reference System | 姿态航向参考系统 |
| ADC | Analog to Digital Converter | 模数转换器 |
| UART | Universal Asynchronous Receiver/Transmitter | 通用异步收发传输器 |
| I2C | Inter-Integrated Circuit | 集成电路总线 |
| SPI | Serial Peripheral Interface | 串行外设接口 |
| Waf | "Waf is not a build system" | 构建系统工具 |
| GTest | Google Test | 谷歌测试框架 |
| CI | Continuous Integration | 持续集成 |
| astyle | Artistic Style | C++ 代码格式化工具 |
| ROS | Robot Operating System | 机器人操作系统 |
| DDS | Data Distribution Service | 数据分发服务 |
| GDB | GNU Debugger | GNU 调试器 |
| JTAG | Joint Test Action Group | 联合测试行动组 |
| SWD | Serial Wire Debug | 串行线调试 |
| OpenOCD | Open On-Chip Debugger | 开源芯片调试器 |
| RTOS | Real-Time Operating System | 实时操作系统 |
| PR | Pull Request | 拉取请求 |

---

*本文档基于 ArduPilot 项目编写，最后更新于 2026年3月*
