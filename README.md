# All-Directional Wheel Chassis (全向轮底盘)

> 基于 STM32H723 的四轮 X 型全向移动机器人底盘控制系统，面向 RoboMaster 竞赛场景。

[![Platform](https://img.shields.io/badge/platform-STM32H723xx-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32h723-733.html)
[![Language](https://img.shields.io/badge/language-C11-orange)](#)
[![Build](https://img.shields.io/badge/build-CMake-green)](#)
[![IDE](https://img.shields.io/badge/IDE-CLion-purple)](#)
[![License](https://img.shields.io/badge/license-MIT-lightgrey)](#)

---

## 📖 项目简介

本项目是一个**四轮 X 型全向轮底盘**的完整运动控制系统。基于 STM32H723xx (ARM Cortex-M7) 微控制器，通过 FDCAN 总线驱动四台 DJI M3508 电机（搭配 C620 电调），实现全向移动、小陀螺、斜坡补偿、功率管理、底盘跟随等 RoboMaster 竞赛所需的核心功能。

代码采用**分层架构**设计，将算法、设备驱动、硬件抽象清晰分离，便于移植、调试和二次开发。

---

## 🗂️ 项目结构

```
Omni_Wheel_Chassis/
├── HLY/                                # 🧠 核心算法与驱动库
│   ├── Inc/                            # 头文件
│   │   ├── alg_PID.h                   #   PID 控制器
│   │   ├── alg_filter.h                #   数字滤波器 (Fourier FIR / Kalman)
│   │   ├── alg_finite_state_machine.h  #   有限状态机 (FSM)
│   │   ├── alg_queue.h                 #   循环队列
│   │   ├── alg_timer.h                 #   软件定时器
│   │   ├── alg_rotation_matrix.h       #   旋转矩阵与 IMU 数据处理
│   │   ├── alg_omni_wheel_chassis.h    #   全向底盘运动学 / 动力学
│   │   ├── alg_spin_top.h              #   小陀螺运动控制
│   │   ├── alg_torque_compensation.h   #   扭矩补偿 (坡道 / 阻力)
│   │   ├── alg_power_management.h      #   功率估计与功率限制
│   │   ├── alg_slope_constrained_planner.h  # 斜坡约束规划器
│   │   ├── alg_chassis_following.h     #   底盘跟随云台控制
│   │   ├── app_keyboard_control.h      #   键鼠控制与系统状态机
│   │   ├── dvc_DJI_M3508.h            #   DJI M3508 电机设备驱动
│   │   ├── drv_can.h                   #   CAN 总线驱动
│   │   ├── drv_math.h                  #   数学工具库
│   │   └── drv_wdg.h                   #   独立看门狗驱动
│   └── Src/                            # 源文件 (对应上述 .h)
├── Core/                               # STM32CubeMX 生成的 HAL 层代码
│   ├── Inc/
│   └── Src/                            # main.c, stm32h7xx_it.c, fdcan.c, ...
├── Drivers/                            # STM32 HAL 库 & CMSIS
│   ├── CMSIS/
│   └── STM32H7xx_HAL_Driver/
├── cmake/                              # CMake 构建配置
│   └── stm32cubemx/
├── MicroLink_H7.cfg                    # 调试器配置文件
├── All_Directional_Wheel_Chassis.ioc   # STM32CubeMX 工程文件
└── CMakeLists.txt                      # 顶层 CMake 构建脚本
```

---

## 🏗️ 分层架构

```
┌─────────────────────────────────────────┐
│           Application (应用层)           │
│   app_keyboard_control                  │
│   键鼠策略 / 系统状态机 / 模式切换       │
├─────────────────────────────────────────┤
│           Algorithm (算法层)             │
│   alg_omni_wheel_chassis  (运动学)      │
│   alg_spin_top            (小陀螺)      │
│   alg_torque_compensation (扭矩补偿)     │
│   alg_power_management    (功率管理)     │
│   alg_chassis_following   (底盘跟随)     │
│   alg_slope_constrained_planner (规划)  │
│   alg_PID                 (PID 控制)    │
│   alg_filter / FSM / Queue / Timer      │
├─────────────────────────────────────────┤
│           Device (设备驱动层)            │
│   dvc_DJI_M3508          (M3508 电机)   │
├─────────────────────────────────────────┤
│           Driver (硬件抽象层)            │
│   drv_can  /  drv_math  /  drv_wdg     │
├─────────────────────────────────────────┤
│           Hardware (硬件层)              │
│   STM32H723xx + FDCAN + TIM + IWDG     │
└─────────────────────────────────────────┘
```

### 各层职责

| 层级 | 前缀 | 职责 | 典型文件 |
|------|------|------|----------|
| **应用层** | `app_` | 顶层业务逻辑、控制策略、模式管理 | `app_keyboard_control` |
| **算法层** | `alg_` | 平台无关的数学/控制算法 | `alg_PID`, `alg_omni_wheel_chassis` |
| **设备层** | `dvc_` | 特定外设的数据解析与控制 | `dvc_DJI_M3508` |
| **驱动层** | `drv_` | 硬件接口封装、底层通信 | `drv_can`, `drv_math` |

---

## 🔧 核心模块详解

### 1. 全向底盘运动学/动力学 (`alg_omni_wheel_chassis`)

四轮 X 型全向底盘的核心解算模块，支持：

- **逆运动学 (Inverse Kinematics)**：将底盘目标速度矢量 $(v_x, v_y, \omega)$ 分解为四个轮子的目标角速度，基于 45° 安装夹角进行解耦
- **逆动力学 (Inverse Dynamics)**：将底盘目标受力/力矩 $(F_x, F_y, M_z)$ 分配为四个轮子的目标扭矩
- **防饱和限幅**：内置电机最大角速度/扭矩约束

### 2. DJI M3508 电机驱动 (`dvc_DJI_M3508`)

- 通过 **FDCAN** 总线与 C620 电调通信
- 支持多种控制模式：**开环 / 电压 / 电流 / 扭矩 / 速度 / 角度**
- 内置级联 PID（角度环 → 速度环 → 扭矩环）
- 实时反馈：编码器角度、转速、扭矩电流、温度
- 通信超时检测与掉线保护

### 3. PID 控制器 (`alg_PID`)

工业级 PID 实现，集成多种高级特性：

| 特性 | 说明 |
|------|------|
| **微分先行 (D-First)** | 仅对反馈值微分，避免设定值突变引起的微分冲击 |
| **变速积分** | 误差大时减弱积分作用，减少超调 |
| **积分分离** | 误差较大时完全关闭积分，防止积分饱和 |
| **死区控制** | 小误差范围内不输出，避免抖振 |
| **前馈控制** | 独立的 $K_F$ 前馈通道，提升响应速度 |
| **输出/积分限幅** | 独立的输出与积分量限幅 |

### 4. 小陀螺控制 (`alg_spin_top`)

RoboMaster 经典战术动作——底盘原地自转的同时保持云台指向目标：

- 根据云台偏航角反馈实时调整底盘自转目标
- 前馈延迟补偿，提升高转速下的跟踪精度

### 5. 扭矩补偿 (`alg_torque_compensation`)

多层次的力矩前馈补偿，提升底盘在复杂地形下的控制精度：

- **坡道补偿**：利用 IMU 姿态角估计重力分量，计算上坡所需的额外力矩
  - 低配版：基于质心高度估计
  - 高配版：基于二次多项式阻力模型
- **阻力补偿**：
  - 低配版：恒定阻力扭矩 + 一阶连续化
  - 高配版：二次多项式角速度-阻力曲线

### 6. 功率管理 (`alg_power_management`)

满足 RoboMaster 裁判系统功率限制的核心模块：

- **功率估计**：
  - 基础模型：$P = k_1 \cdot |\omega \cdot I| + k_2 \cdot I^2 + P_0$
  - Maclaurin 展开模型：$P = k_0 + k_1 I + k_2 \omega + k_3 I \omega + k_4 I^2 + k_5 \omega^2$
- **PID 闭环功率衰减**：当估计功率超出裁判系统限制时，自动计算衰减系数
- **分离式功率限制**：区分动力学解算电流与辅助补偿电流，按优先级分别限制

### 7. 底盘跟随 (`alg_chassis_following`)

PID 闭环控制底盘偏航角跟随云台指向，使底盘始终朝向云台瞄准方向。

### 8. 数字滤波器 (`alg_filter`)

- **Fourier FIR 滤波器**：可配置低通/高通/带通/带阻，支持任意阶数，基于窗函数法设计
- **Kalman 滤波器**：一维卡尔曼，适合实时传感器数据融合

### 9. 基础算法库

| 模块 | 功能 |
|------|------|
| `alg_rotation_matrix` | 2D/3D 旋转矩阵、欧拉角解算、IMU 数据处理、斜坡姿态估计 |
| `alg_slope_constrained_planner` | 斜坡函数规划器，平滑目标值变化，支持对称/非对称加减速 |
| `alg_finite_state_machine` | 通用有限状态机框架，支持最多 10 状态 |
| `alg_queue` | 动态循环队列，用于滑动窗口滤波等场景 |
| `alg_timer` | 毫秒级软件定时器，支持 Reset/Wait/Trigger/Timeout 状态 |

### 10. 应用层：键鼠策略控制 (`app_keyboard_control`)

RoboMaster 操作手键鼠控制的完整状态机：

- **底盘控制**：WASD 全向移动、Q/E 自转、Shift 加速
- **功率模式切换**：常规 / 加速 / 爆发 (Burst)
- **自瞄模式切换**：装甲板 / 能量机关
- **底盘优先级属性**：血量优先 / 功率优先
- **发射机构优先级**：冷却优先 / 爆发优先
- **裁判系统/超级电容可信标志位管理**

---

## 🚀 快速开始

### 硬件要求

| 组件 | 型号 |
|------|------|
| 主控 | STM32H723xx (ARM Cortex-M7) |
| 电机 ×4 | DJI M3508 + C620 电调 |
| 通信 | FDCAN 总线 |
| IMU | 9 轴 / 6 轴 IMU (加速度计 + 陀螺仪 + 磁力计) |
| 调试器 | MicroLink / J-Link / ST-Link |

### 软件依赖

- **CMake** ≥ 3.22
- **ARM GCC 工具链** (`arm-none-eabi-gcc`)
- **STM32CubeMX** (用于 HAL 层代码生成)
- **CLion** (推荐) 或 VS Code + Cortex-Debug

### 构建步骤

```bash
# 1. 克隆仓库
git clone <your-repo-url>
cd Omni_Wheel_Chassis

# 2. 使用 CMake 配置构建
mkdir build && cd build
cmake .. -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=<path-to-arm-toolchain.cmake>

# 3. 编译
make -j$(nproc)
```

### 在 CLion 中打开

1. 使用 CLion 打开项目根目录
2. CLion 会自动识别 `CMakeLists.txt`
3. 在 Settings → Build, Execution, Deployment → Toolchain 中配置 ARM GCC 工具链
4. 构建并下载至目标芯片

### 硬件连接

```
STM32H723           C620 电调 (×4)
  FDCAN1 TX ──────── CAN 总线
  FDCAN1 RX ──────── CAN 总线
  FDCAN2 TX ──────── CAN 总线 (备用)
  FDCAN2 RX ──────── CAN 总线 (备用)
  IWDG        ────── 独立看门狗 (可选)
```

### CAN ID 分配

| 电机位置 | CAN ID |
|----------|--------|
| 左前 (LF) | 0x201 |
| 左后 (LB) | 0x202 |
| 右后 (RB) | 0x203 |
| 右前 (RF) | 0x204 |

---

## 📊 数据流

```
操纵手键鼠输入
      │
      ▼
┌─────────────────┐
│ app_keyboard_   │  键鼠解析 → 系统状态机 → 底盘/发射机构目标指令
│ control         │
└───────┬─────────┘
        │ chassis_target (vx, vy, omega)
        ▼
┌─────────────────┐
│ spin_top /      │  小陀螺修正 / 底盘跟随
│ chassis_follow  │
└───────┬─────────┘
        │ corrected_target
        ▼
┌─────────────────┐
│ Inverse         │  逆运动学 / 逆动力学 → 四轮目标角速度 / 扭矩
│ Kinematics      │
│ / Dynamics      │
└───────┬─────────┘
        │ wheel_targets
        ▼
┌─────────────────┐
│ torque_         │  坡道前馈 / 阻力补偿
│ compensation    │
└───────┬─────────┘
        │ compensated_torque
        ▼
┌─────────────────┐
│ power_          │  功率估计 → PID 衰减 → 限幅输出
│ management      │
└───────┬─────────┘
        │ final_torque_cmd
        ▼
┌─────────────────┐
│ DJI_M3508       │  级联 PID → CAN 发送 → C620 电调 → 电机
│ (Motor Driver)  │
└─────────────────┘
```

---

## 🧪 关键参数配置

### 底盘物理参数 (`Struct_Chassis_Config`)

```c
float r_center;   // 机器人中心到车轮中心的距离 (单位: m)
float R;          // 车轮半径 (单位: m)
float max_omega;  // 电机最大允许角速度 (单位: rad/s)
float max_torque; // 电机最大允许扭矩 (单位: N·m)
```

### PID 调参指南

每个 M3508 电机内置**角度-速度级联 PID**：

1. **速度环**：先整定 $K_P$ 使速度响应快速无震荡，再加入较小的 $K_I$ 消除稳态误差
2. **角度环**：整定 $K_P$ 使位置跟踪平滑，避免超调
3. **前馈**：$K_F$ 可提高对动态目标的跟踪能力

---

## 📝 开发日志

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 1.5 | - | 基础框架搭建 |
| 1.6 | - | PID 控制器完善 |
| 1.7 | - | 全向底盘运动学/动力学 |
| 1.8 | - | 功率管理 / 扭矩补偿 |
| 1.9 | - | 键鼠策略控制 / 看门狗驱动 |

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

### 代码规范

- 文件命名：`{层级}_{模块}.{h/c}`（如 `alg_PID.h`、`drv_can.c`）
- 类型命名：`Struct_{模块}` 结构体、`Enum_{类型}` 枚举
- 函数命名：`{模块}_{动作}_{子功能}()`（如 `DJI_M3508_Set_Target_Omega`）
- 统一使用 `float` 类型进行浮点运算

---

## 📄 许可证

本项目采用 [MIT License](LICENSE)。

---

## 🙏 致谢

- [STMicroelectronics](https://www.st.com/) — STM32H7 系列 MCU 与 HAL 库
- [DJI](https://www.dji.com/) — RoboMaster M3508 电机与 C620 电调
- [RoboMaster](https://www.robomaster.com/) — 竞赛平台

---

> **HLY** | Built with ❤️ for RoboMaster
