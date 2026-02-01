# RoboMaster 步兵机器人

![Build Status](https://img.shields.io/badge/Build-Keil_MDK-brightgreen)
![Platform](https://img.shields.io/badge/Platform-STM32F4-blue)
![License](https://img.shields.io/badge/License-AGPL-orange)

## 📖 项目简介

本项目为 RoboMaster 步兵机器人的嵌入式底层控制代码。基于 **STM32F427** 平台，采用 **裸机轮询 (Bare-Metal)** 架构，通过定时器分时调度实现多任务处理。


## 🏗 软件架构

本项目采用 **“定时器中断 + 主循环状态机”** 的轻量级架构，确保控制周期的确定性。

### 核心任务调度 (`robot_status.c`)

系统时基由 `TIM2` (1ms) 提供，通过标志位累加触发不同频率的任务：

| 任务名称 | 频率 (Hz) | 优先级 | 功能描述 |
| :--- | :--- | :--- | :--- |
| **遥控解析** | 100 | High | 解析 DBUS 数据，看门狗喂狗 |
| **云台控制** | ~33 | High | Pitch (达妙) / Yaw (GM6020) PID 计算 |
| **底盘控制** | 50 | Medium | 麦轮运动解算、功率限制策略 |
| **UI 刷新** | 动态 | Low | 裁判系统图形交互绘制 |

### 文件结构

```text
RoboMaster-Infantry/
├── Core/                     # STM32核心文件
│   ├── main.c                    # 主程序入口
│   ├── system_stm32f4xx.c        # 系统时钟配置
│   ├── stm32f4xx_it.c            # 中断服务程序
│   ├── stm32f4xx_hal_msp.c       # MCU特定包初始化
│   ├── gpio.c                    # GPIO配置
│   ├── usart.c                   # 串口通信配置
│   ├── tim.c                     # 定时器配置
│   ├── can.c                     # CAN总线配置
│   ├── dma.c                     # DMA配置
│   └── wwdg.c                    # 窗口看门狗配置
├── Periphs/                  # 外设驱动层
│   ├── can_rx.c                  # CAN接收处理
│   ├── can_tx.c                  # CAN发送控制
│   ├── dbus.c                    # 遥控器DBUS协议解析
│   ├── hwt906.c                  # 维特智能IMU驱动
│   ├── referee.c                 # 裁判系统基础驱动
│   ├── rm_referee.c              # 裁判系统协议解析
│   ├── crc_ref.c                 # CRC校验算法
│   ├── filter.c                  # 滤波算法
│   ├── pid.c                     # PID控制算法
│   ├── referee_task.c            # 裁判系统任务管理
│   └── referee_UI.c              # 自定义UI绘图引擎
├── RTT/                      # SEGGER RTT调试组件
│   ├── SEGGER_RTT.c              # RTT核心库实现
│   └── SEGGER_RTT_printf.c       # RTT格式化输出
├── control/                  # 应用层
│   ├── robot_status.c           # 任务调度器（核心控制器）
│   ├── chassis_control.c        # 底盘运动控制
│   ├── gimbal_control.c         # 云台控制
│   └── rc_control.c             # 遥控器模式映射
├── Drivers/                  # STM32 HAL驱动库
│   ├── CMSIS/                   # Cortex-M软件接口标准
│   └── STM32F4xx_HAL_Driver/    # STM32F4 HAL驱动
└──                        

```

---

## 🔌 硬件拓扑与配置

> **⚠️ 警告：本项目使用了非默认的硬件 ID，上电前请务必核对拨码开关和电机参数！**

### 1. CAN 总线配置

| 总线 | 设备 | ID (Hex) | 宏定义 | 备注 |
| :--- | :--- | :--- | :--- | :--- |
| **CAN1** | 底盘电机 LF | `0x201` | `MOTOR_CHASSIS_1` | M3508 |
| **CAN1** | 底盘电机 RF | `0x202` | `MOTOR_CHASSIS_2` | M3508 |
| **CAN1** | 底盘电机 LB | `0x203` | `MOTOR_CHASSIS_3` | M3508 |
| **CAN1** | 底盘电机 RB | `0x204` | `MOTOR_CHASSIS_4` | M3508 |
| **CAN1** | 超级电容 | `0x211` | `SUPERCAP` | 非通用 ID (0x210) |
| **CAN2** | 拨弹电机 | `0x205` | `MOTOR_FEEDER` | M2006 |
| **CAN2** | 摩擦轮 L | `0x206` | `MOTOR_FRICTION` | M3508 |
| **CAN2** | 摩擦轮 R | `0x207` | `MOTOR_FRICTION` | M3508 |
| **CAN2** | Yaw 轴 | `0x209` | `MOTOR_YAW` | GM6020 (拨码需设为 5) |
| **CAN2** | Pitch 轴 | `0x10C` | N/A | 达妙电机 (MIT模式) |

### 2. 串口 (UART) 配置

- **USART1** (921600 bps)：连接 **维特智能 HWT906** 陀螺仪 (DMA 接收)。
- **USART3** (100000 bps)：连接 C 型板 SBUS 接收机 (DBUS 协议)。
- **USART6** (115200 bps)：连接裁判系统图传模块。

---

## 🚀 快速开始

### 1. 环境准备

- **IDE**：Keil uVision 5 或其他支持 ARM 的 IDE。
- **调试工具**：J-Link ST-Link


### 2. 操作说明

- **S1 开关 (右侧)**
    - 上：键盘鼠标模式 (`KB_CONTROL`)
    - 中：遥控器跟随模式 (`RC_CONTROL`)
    - 下：小陀螺模式 (`SPIN_CONTROL`)

- **S2 开关 (左侧)**
    - 下：开启摩擦轮 (允许射击)
    - 上：安全模式 (停止摩擦轮)

---
### 3.关键事项
       因为硬件原因，HWT906 IMU暂时替换为HWT606 IMU，相关代码也是适配HWT606 IMU。
---