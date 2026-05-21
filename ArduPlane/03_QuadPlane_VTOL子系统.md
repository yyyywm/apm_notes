# 03 — QuadPlane VTOL 子系统

> 文件: `ArduPlane/quadplane.h/.cpp` + `transition.h` + `tailsitter.h` + `tiltrotor.h` + `VTOL_Assist.h` + `qautotune.h`
> 核心: QuadPlane 类管理所有 VTOL/垂直起降功能，条件编译 (`HAL_QUADPLANE_ENABLED`)

---

## 推力类型 (ThrustType)

```
SLT         — 独立升力推力 (前拉电机 + 多旋翼电机分开)
TAILSITTER  — 尾座式 (机身竖立，舵面 + 电机)
TILTROTOR   — 倾转旋翼 (电机/旋翼可倾转，伺服或二进制)
```

---

## 位置控制状态机

```
QPOS_NONE
  → QPOS_APPROACH          进近阶段
    → QPOS_AIRBRAKE        气动减速
      → QPOS_POSITION1     第一定位点
        → QPOS_POSITION2   第二定位点 (精确)
          → QPOS_LAND_DESCEND  下降
            → QPOS_LAND_FINAL  末段
              → QPOS_LAND_COMPLETE  着陆完成

特殊: QPOS_LAND_ABORT → 中止降落，回到悬停
```

---

## 降落进场阶段 (VTOLApproach::Stage)

```
RTL
 → LOITER_TO_ALT     飞到盘旋高度
 → ENSURE_RADIUS     确保在半径内
 → WAIT_FOR_BREAKOUT 等待进入进场线
 → APPROACH_LINE     沿进场线飞行
 → VTOL_LANDING      切换到 VTOL 着陆
```

---

## 过渡状态机 (VTOL ↔ FixedWing)

### SLT_Transition (独立升力推力，最常见)

```
状态: AIRSPEED_WAIT → TIMER → DONE

1. AIRSPEED_WAIT — 等待达到最小空速
2. TIMER — 空速达标后计时 (Q_TRANSITION_MS)
3. DONE — 过渡完成，关闭多旋翼电机
```

### Tailsitter_Transition

```
状态: ANGLE_WAIT_FW → ANGLE_WAIT_VTOL → DONE

基于姿态角度而非空速来判断过渡完成
```

### Tiltrotor_Transition

```
继承 SLT_Transition，额外覆盖:
- update_yaw_target() — 倾转偏航目标
- show_vtol_view() — VTOL 视图条件
- allow_vfwd() — 前向速度允许条件
```

---

## VTOL_Assist — 固定翼模式的四旋翼辅助

当飞机在固定翼模式下遇到危险时，四旋翼电机自动介入:

| 辅助类型 | 触发条件 | 参数 |
|---------|---------|------|
| Speed Assist | 空速 < 阈值 | `Q_ASSIST_SPEED` |
| Angle Assist | 姿态误差 > 阈值 | `Q_ASSIST_ANGLE` |
| Altitude Assist | 检测到掉高 | `Q_ASSIST_ALT` |
| Force Assist | 遥控器开关 | `Q_ASSIST_FORCE` 通道 |

每种 Assist 有独立的滞回和延迟 (`Q_ASSIST_DELAY`) 防抖。

---

## 关键参数 (Q_* 系列)

| 参数 | 说明 |
|------|------|
| `Q_ENABLE` | 主开关 (0:关, 1:开, 2:启用 VTOL AUTO) |
| `Q_FRAME_CLASS` | 机架类型 (Quad/Hexa/Octa/Tri/Y6) |
| `Q_FRAME_TYPE` | 机架布局 (X/+/H/V) |
| `Q_TRANSITION_MS` | 过渡等待时间 ms |
| `Q_PILOT_SPD_UP/DN` | 飞手升降速率 cm/s |
| `Q_LAND_FINAL_SPD` | 着陆末段下降速度 |
| `Q_LAND_FINAL_ALT` | 着陆末段起始高度 |
| `Q_ASSIST_SPEED` | 辅助触发空速 m/s |
| `Q_ASSIST_ANGLE` | 辅助触发角度 deg |
| `Q_VFWD_GAIN` | 前向速度保持增益 |
| `Q_OPTIONS` | 23+ 位掩码选项 |
| `Q_TAILSIT_*` | Tailsitter 系列参数 |
| `Q_TILT_*` | Tiltrotor 系列参数 |

---

## QuadPlane 拥有的子系统

```
QuadPlane
  ├── AP_InertialNav    惯性导航
  ├── AP_MotorsMulticopter*     电机混控
  ├── AC_AttitudeControl_Multi* 姿态控制 (多旋翼)
  ├── AC_PosControl*            位置控制
  ├── AC_WPNav*                 航点导航 (VTOL)
  ├── AC_Loiter*                悬停导航
  ├── AC_WeatherVane*           风向标校正
  ├── Transition*   → 过渡状态机 (多态)
  ├── Tailsitter    尾座式特定
  ├── Tiltrotor     倾转旋翼特定
  ├── VTOL_Assist   VTOL 辅助
  └── QAutoTune     VTOL 自动调参
```

---

## 相关笔记

- [QuadPlane_4+1_Auto_全流程解析](../QuadPlane_4+1_Auto_全流程解析.md)
- [QuadPlane_update_分支逻辑详解](../QuadPlane_update_分支逻辑详解.md)
- [SLT_Transition_状态切换详解](../SLT_Transition_状态切换详解.md)
- [VTOL_Control_System_Architecture](../VTOL_Control_System_Architecture.md)
- [VTOL_Landing_Logic](../VTOL_Landing_Logic.md)
- [VTOL_完整起降流程代码解析](../VTOL_完整起降流程代码解析.md)
- [Tailsitter参数解读](../Tailsitter参数解读_2022-10-17_Pixhawk1.md)

---

## 待补充

- [ ] control_auto() 完整流程解析
- [ ] vtol_position_controller() PID 详解
- [ ] Q_OPTIONS 每个位的作用
- [ ] 过渡过程中油门/舵面的协调策略
