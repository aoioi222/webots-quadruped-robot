# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

Webots 四足机器人（机械狗）控制器，C 语言编写，使用 Webots C API。控制 12 个电机（4腿×3关节），实现对角小跑（Trot）等多种步态。

## 构建与运行

```bash
# 在 Webots 中直接运行（会自动编译）
# 或手动编译：
cd controllers/my_controller
make

# 清理编译产物
make clean
```

编译输出到 `build/release/my_controller.exe`。`Makefile` 中 `C_SOURCES` 列出了所有需要编译的源文件，新增 `.c` 文件后需手动添加。

## 架构：4层控制流水线

```
速度指令 → [步态生成器] → [步态调度器] → [步态控制器] → [腿部状态机×4] → 电机输出
                ↑                                                      ↑
           gait_generator                                     leg_state_machine
                ↓                                                      ↓
           gait_scheduler                                      bezier_foot_tra (摆动轨迹)
                ↓                                                      ↓
           gait_controller                                   IK/FK/Jacobian (运动学解算)
                                                                        ↑
                                                               body_control (姿态PD修正)
```

### 各层职责

| 模块 | 文件 | 职责 |
|------|------|------|
| 步态生成器 | `gait_generator.c/h` | 根据速度指令和步态类型（Trot/Walk/Pronk/Pace/Bound），计算全局相位、各腿摆动/支撑起止点 |
| 步态调度器 | `gait_scheduler.c/h` | 连接步态生成器与状态机，根据 `need_swing` 标志触发支撑↔摆动状态切换 |
| 步态控制器 | `gait_controller.c/h` | 总协调：初始化所有模块，计算各腿相位（Trot偏移：RF/LB同相，LF/RB同相），驱动状态机更新 |
| 腿部状态机 | `leg_state_machine.c/h` | 每腿独立状态机（HOLD/STANCE/SWING/WHEEL/FOLD/UNFOLD/ERROR），支撑相线性插值，摆动相贝塞尔轨迹 |

### 运动学与轨迹

| 模块 | 文件 | 职责 |
|------|------|------|
| 正运动学 | `Forward_kinematics_calculate.c/h` | 变换矩阵法：关节角度 → 足端位置 |
| 逆运动学（几何法） | `Inverse_kinematics_calculate.c/h` | 几何法解析IK：足端位置 → 关节角度 |
| 逆运动学（雅可比法） | `Jacobian_matrix.c/h` | 雅可比迭代IK、伪逆、速度/力映射。统一入口 `leg_ik_solve()` 支持几何法/雅可比/自动降级 |
| 贝塞尔轨迹 | `bezier_foot_tra.c/h` | 三次贝塞尔曲线生成摆动轨迹，离线离散 → 实时查表插值 |
| 足端结构定义 | `Foot_structure.h` | 机械尺寸常量、关节限位、腿ID枚举、坐标系统、`FootPosition_t`/`LegJointAngles_t` 等核心类型 |

### 机体控制与状态

| 模块 | 文件 | 职责 |
|------|------|------|
| 姿态控制 | `body_control.c/h` | PD控制器修正pitch/roll：在支撑腿足端Z高度叠加修正量，使机身保持水平 |
| 机器人状态 | `robot_state.c/h` | 全局状态管理（`g_robot_state`）：模式切换、速度指令、IMU数据、机身姿态 |
| 头部聚合 | `include_head.h` | 聚合所有 Webots API 头和项目头文件 |
| 基础定义 | `head.h` | 数学常量（π、弧度转换）、C标准库头文件 |

### 主循环（`my_controller.c`）

```
wb_robot_step (TIME_STEP=10ms):
  1. 读取 IMU → g_robot_state.body_pitch/roll/yaw
  2. gait_gen_update()        — 计算全局相位、步长、各腿 swing/stance 目标
  3. gait_scheduler_update()  — 根据 need_swing 触发状态切换
  4. body_attitude_control_update() — 支撑腿 Z 高度 PD 修正
  5. gait_control_update()    — 计算各腿相位 → 状态机更新 → IK 解算 → 方向重映射
  6. set_all_motor()          — 下发关节角度到电机
```

### 数据流关键全局变量（定义在 `gait_controller.c`）

- `gait_gen` — 步态生成器上下文
- `gait_scheduler` — 步态调度器上下文
- `leg_fsm[4]` — 四条腿状态机
- `foot_target_pos[4]` — 各腿当前目标足端位置
- `leg_target_joints[4]` — 各腿当前目标关节角度
- `g_robot_state` — 全局机器人状态（定义在 `robot_state.c`）

## 坐标系统与腿编号

1.机体坐标系为前为X轴正方向，左为Y轴正方向，上为Z轴正方向

2.髋展关节（joint1）旋转轴为前后，髋转关节（joint2）旋转轴为左右，膝关节（joint3）旋转轴为左右

3.joint1和joint2之间的连杆长度为0.023，大腿长度为0.10,小腿长度为0.10

4.当各个关节的角度都为0时，
左腿足端在机体坐标系减去腿部偏移后的坐标为（0，0.023，-0.2）
右腿足端在机体坐标系减去腿部偏移后的坐标为（0，-0.023，-0.2）【仿真建模的结果】

5.定义从关节的旋转轴方向的外侧看过去，关节顺时针旋转为正角度，逆时针旋转为负角度


```
     左后(3)          左前(1)
        |               |
        |    机头→      |
     右后(2)          右前(0)
```

腿ID：`LEG_R_F=0, LEG_L_F=1, LEG_R_B=2, LEG_L_B=3`
##欧拉角
body_roll向左倾斜是负，向右倾斜是正
body pitch向上仰头是负，向下低头是正
body_yaw向右转是负，向左转是正
## 调参关键位置

- **步态参数**：`gait_controller.h` — `CYCLE_TIME`, `SWING_RATIO`, `STEP_HEIGHT`, `MAX_STEP_X/Y`, `CMD_V_X/Y/YAW`
- **步态切换**：`gait_controller.h` 中 `#define TEST_TROT` 切换为 `TEST_WALK` 等
- **姿态PD**：`body_control.c` — `ATTITUDE_PD_DEFAULT`（kp_pitch, kd_pitch, kp_roll, kd_roll, max_correction）
- **机械尺寸**：`Foot_structure.h` — `THIGH_LENGTH`, `CALF_LENGTH`, 髋关节偏移量, `FOOT_OFFSET_X/Y/Z`
- **关节限位**：`Foot_structure.h` — `JOINT1_MIN/MAX`, `JOINT2_MIN/MAX`, `JOINT3_MIN/MAX`
- **贝塞尔轨迹**：`bezier_foot_tra.h` — `CONFIG_DEFAULT_LIFT_HEIGHT`

## 电机方向重映射

Webots 电机安装方向与 IK 输出符号可能不一致，`leg_direction_remap()`（在 `gait_controller.c` 和 `my_controller.c` 中各有一份副本）负责处理各腿的符号翻转。如果某条腿关节运动方向反了，修改此函数中对应腿的符号。
此外那些数据数据在套一层的大多是因为webots仿真的算法的误差问题

## 空壳文件

以下文件目前为空或仅含占位代码，尚未实现功能：
- `error_handler.c/h` — 错误处理
- `mode_manager.c/h` — 模式管理
- `task_scheduler.c` — 任务调度
