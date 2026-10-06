# 26_Hero_Chassis 英雄机器人 · 底盘板电控

## 简介
- 项目：RoboMaster 英雄机器人底盘板（电控）
- MCU：STM32H723VGTx
- 系统：FreeRTOS（CMSIS-RTOS v2）+ STM32Cube HAL
- 通信：FDCAN1 / FDCAN2 / FDCAN3、裁判系统串口（USART1）
- 状态：比赛验证版，已完成“能跑、能讲、能传承”的模块化整理（行为保持不变）

## 主要功能
- 底盘控制：4×M3508 麦克纳姆轮解算 + 速度环 PID + 电流分配
- 功率控制：功率模型 P = k1·ω·I + k2·I² + k3·ω²，结合缓冲能量分配电流
- 超级电容：CAN 控制/回读、能量缓冲、离线保护
- 上坎：前/后气缸状态机 + 比例阀 PWM
- 板间链路：接收云台板整机指令（0x300）与 UI 数据（0x308），回传底盘状态（0x304）
- 拨盘转发：底盘板只做透传（0x303→0x207、0x301→0x07、0x17→0x302），拨盘闭环由云台板完成
- 裁判系统：串口解析 + 底盘 UI 绘制 + WS2812 状态灯
- 安全：云台板链路 200ms 离线、底盘输出被裁判系统切断时整机失能

## 目录结构
- `Core/`：HAL 外设初始化、`main.c`、FreeRTOS 配置
- `Algorithm/`：PID（全项目统一一套）、Ramp、Mahony、EKF、Kalman、LADRC、滤波等算法
- `BSP/`：FDCAN、功率计、超级电容、裁判系统串口（drv_uart）、ANO 协议、蜂鸣器等板级驱动
- `Devices/`：BMI088 IMU、WS2812 等器件驱动、电机通用结构体(motor.h)
- `Task/`：控制(control_task)、CAN接收(can_task)、打印(print_task)、IMU、device、UI、monitor、supercap 等 FreeRTOS 任务
- `Modules/`：整机调度(robot_control)、底盘(chassis_control)、上坎(climb_control)、功率(power_control)、超电决策(supercap_control)、CAN收发(can_transmit/can_receive)、板间链路(gimbal_link)、裁判系统(RM_JudgeSystem)
- `Drivers/ Middlewares/`：ST HAL、CMSIS、FreeRTOS

## 编译与烧录
1. 用 Keil MDK 打开 `MDK-ARM/h723_cloud.uvprojx`
2. 编译（应 0 Error；Warning 仅来自 Keil 自带 CMSIS-DSP 库）
3. 通过 J-Link / DAP 烧录到底盘板

## 编码约定
- 现有 `.c/.h` 源文件为 **GBK** 编码，请勿用会批量转换编码的工具处理
- `README.md` 与 `docs/` 下的文档为 UTF-8

## CAN 分配（关键）
| 总线 | 方向 | ID / 说明 |
|---|---|---|
| FDCAN1 | 收 | 0x300 云台整机指令、0x308 UI 数据、0x303 拨盘速度、0x301 拨盘模式 |
| FDCAN1 | 发 | 0x304 底盘状态（弹速/热量/冷却/颜色/裁判在线）、0x302 拨盘反馈 |
| FDCAN2 | 发 | 0x200 底盘电机电流、0x601 超级电容指令 |
| FDCAN2 | 收 | 0x201~0x204 底盘电机反馈、0x301 功率计、0x600 超级电容反馈 |
| FDCAN3 | 发 | 0x207 拨盘速度、0x07 拨盘模式 |
| FDCAN3 | 收 | 0x17 达妙拨盘反馈 |

> 注意：`0x301` 在不同总线上含义不同 —— **FDCAN1 = 拨盘模式指令**，**FDCAN2 = 功率计反馈**，改动前先确认总线。

## 安全须知
- 上电默认失能：`chassis.mode == 0`、云台链路离线（200ms）、或底盘输出被裁判系统切断时，控制循环直接返回，发送 0x200 零电流并让前后气缸全收
- 超级电容离线 2s 判定离线，离线时电流上限自动回落到保守值
- 上电/测试时注意底盘轮子与气缸运动部件

## 历史 / 废弃说明
- `User/Inc/struct_typedef.h`（已删除）：与 `<stdint.h>` 重复的历史类型定义，`fp32/fp64/bool_t` 全项目无引用，相关头文件改用 `<stdint.h>`
- `Modules/Inc`、`Modules/Src`（已取消）：`power_control`、`RM_JudgeSystem` 已与其它模块一样平铺到 `Modules/`
- `Algorithm/pid.c/.h`：全项目统一 PID（底盘速度环、功率缓冲环、IMU 温控共用；旧哈工程移植 PID 已合并移除）