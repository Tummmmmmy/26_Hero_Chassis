# 底盘板电控架构说明

> 本文件描述 `26_Hero_Chassis` 的代码结构与本次工程化重构的结果（“比赛验证版”，已完成实车验证）。

## 1. 现状（重构前）

> 注：整体重构已完成（2026-09），本节保留重构前的状态描述。

重构前应用层逻辑全部集中在 `Core/Src/main.c`（约 1317 行），其中：

- `Chassis_Control_Loop()`（136 行）：双板通信发送、失能/使能切换、上坎、底盘运动、功率控制、CAN 输出
- `Can_Decode_Task()`（125 行）：在任务里直接解析云台指令、底盘电机、功率计、超电、拨盘反馈
- `chassis_pneumatic_update()`（67 行）：气缸状态机
- `BSP_FDCAN_Config()`（29）、`DJI_Motor_Send_Command()`（30）、`Send_Gimbal_Data()`（29）、`HAL_FDCAN_RxFifo0Callback()`（23）等硬件与协议函数
- 自己的 PID 实现：`PID_Init` / `PID_Calculate` / `PID_Cal_Enhanced` / `PID_Cal_Increment`
- `Motor_Init()` / `Robot_Param_Init()` 及各类全局电机对象

也就是说：硬件初始化、CAN 协议打包、控制计算、整机调度全部混在一个文件里。

## 2. 目标架构

应用层按“职责域”拆成模块（统一放在 `Modules/`）：

```text
Robot_Control_Loop()                     ← 整机调度（Modules/robot_control.c, 1ms）
    ├─ 失能检查 / 使能切换 / LED 状态
    ├─ Climb_Control_Update()            ← Modules/climb_control.c（气缸相位）
    ├─ Chassis_Control_Update()          ← Modules/chassis_control.c（麦轮解算 + 速度环）
    ├─ power_control_run()               ← Modules/power_control.c（功率分配）
    └─ 统一 CAN 输出                     ← Modules/can_transmit.c
```

CAN 分三层：

```text
应用层（chassis / climb / power）
        ↓
Modules/can_transmit.c   协议打包：DJI 电流 / 双板状态 / 拨盘转发
        ↓
BSP/bsp_can.c            硬件收发：FDCAN 初始化、Filter、BSP_CAN_Send、RX 中断
        ↓
HAL_FDCAN
```

接收方向：

```text
FDCAN 中断 → BSP/bsp_can.c → osMessageQueuePut(canRxQueueHandle)
     → Task/can_task.c（Can_Decode_Task 取帧）
     → Modules/can_receive.c（按总线/ID 分拣）
     → gimbal_link / chassis_control / power_meter / SuperCap / can_transmit(转发)
```

任务归属：

```text
Task/
├── control_task.c    Chassis_Control_task（1ms 控制任务）
├── can_task.c        Can_Decode_Task
├── monitor_task.c    离线巡检（底盘电机 / 双板链路）
├── device_task.c     WS2812 状态灯
├── UI_task.c         Client_UI_Task（裁判系统 UI 绘制）
├── INS_task.c        IMU 数据生产
└── print_task.c      Print_Entry_Task（预留 VOFA 波形调试）
```

## 3. 数据流

```text
云台板 0x300 / 0x308 ──▶ can_receive ──▶ gimbal_link（chassis / UI_Data 快照）
拨盘 0x17 ──▶ can_receive ──▶ can_transmit（转发 0x302 回云台）
底盘电机 0x201~0x204 ──▶ can_receive ──▶ chassis_control（反馈解码）
功率计 0x301(FDCAN2) ──▶ can_receive ──▶ bsp_power_meter
超级电容 0x600 ──▶ can_receive ──▶ bsp_SuperCap
                                        │
                                        ▼
                Task/control_task.c → Robot_Control_Loop()（1ms）
                      ├─ 失能检查（云台链路 / 裁判系统底盘输出）
                      ├─ Climb_Control_Update()      → 气缸 IO
                      ├─ Chassis_Control_Update()    → Chassis_Output_t（速度环电流）
                      ├─ power_control_run()         → 最终电流（功率 + 超电约束）
                      └─ 0x200 电流 / 0x601 超电 / 0x304 回传 → BSP/bsp_can → FDCAN
```

## 4. 状态归属

| 状态 | 归属 |
|---|---|
| `Motor_Controller` / `Motor_Feedback` / `CAN_Rx_Message_t` | `Core/Inc/main.h`（全项目共用类型） |
| 底盘 4 电机、麦轮解算、速度环 PID、逻辑→物理 ID 映射 | `Modules/chassis_control.c`（电机数组为 `static`，对外只给 `Chassis_Output_t`） |
| 气缸相位 | `Modules/climb_control.c`（`static`，对外只给 `Climb_Get_Phase()`） |
| 功率控制器（缓冲 PID、功率模型参数、分配结果） | `Modules/power_control.c` |
| 超级电容状态 | `BSP/bsp_SuperCap.c`（`g_supercap`） |
| 功率计数据 | `BSP/bsp_power_meter.c`（`power_meter`） |
| 双板指令 / UI 快照、链路在线状态 | `Modules/gimbal_link.c` |
| 拨盘转发（速度/模式/反馈） | `Modules/can_transmit.c`（`Dial_Relay_*`） |
| 系统 LED 状态、失能/使能切换 | `Modules/robot_control.c` |

## 5. 历史 / 废弃代码

- `User/Inc/struct_typedef.h`：与 `<stdint.h>` 重复的历史类型定义（`fp32/fp64/bool_t` 全项目无引用），已删除；`bsp_power_meter.h`、`power_control.h` 改用 `<stdint.h>`。
- `Modules/Inc`、`Modules/Src`：目录已取消，`power_control`、`RM_JudgeSystem` 平铺到 `Modules/`。
- 空目录 `Control/`、`Motor/` 及其 Keil 空分组已删除，包含路径同步清理。
- 死代码清理（Step 9）：LK 云台电机残骸 `gimbal_yaw_motor` / `Motor_Init()`、`main.c` 中未使用的数学与协议宏、双板 `chassis.cnt` 计数（只写不读）、注释掉的旧 VOFA 打印与旧 `Monitor_Task`。
- `Modules/can_receive.c` 中保留了“拨盘力矩转发”的**历史备用路径**（注释形式，同为 0x303）：当前拨盘走速度控制，若改回力矩控制按注释里的写法替换即可。
- `Modules/robot_control.c` 中的 `DATA_ACQUISITION_MODE`（默认 0）：数据采集模式保留，打开后绕过功率控制直接发速度环原始电流并输出 VOFA 波形。
- 主循环里的拨盘速度/模式/反馈转发是**透传**，拨盘闭环在云台板；底盘板不参与其控制。

## 6. 重构节奏

每步以“Keil 0 Error → 烧录 → 实车/上电验证 → git commit”为门槛，不一次性大改：

1. 基线建立（`46458dd`）
2. 模块骨架，只搬归属明确的类型 ✅（`8948e48`）
3. PID 统一为 `Algorithm/pid.h/.c` ✅（`d81d5e3`）
4. CAN 硬件层 `BSP/bsp_can` ✅（`3199bba`）
5. CAN 发送层 `Modules/can_transmit` ✅（`d42048e`）
6. `Modules/chassis_control`（麦轮解算 + 速度环） ✅（`9324038`）
7. `Modules/climb_control`（气缸） ✅（`e0a3a65`）
8. `Modules/can_receive` + `Task/can_task` + `Modules/gimbal_link` ✅（`ca1eeaa`）
9. `Modules/robot_control` + `Task/control_task` ✅（`1e9ae3f`）
10. 死代码清理 ✅（`71e06f9`）
11. 目录/命名整理 + README + 架构文档（本步）

## 7. 编码约定

- 源码（`.c/.h`）保持 GBK，不随重构转换编码。
- 文档（`.md`）为 UTF-8。
- 注释用中文，简洁工程风格：核心函数写清作用与参数，简单 getter 一行即可。

## 8. 完成状态（2026-09）

```text
Robot_Control_Loop (Modules/robot_control.c, 1ms 调度)
    ├─ 失能检查 / 使能切换    Modules/robot_control.c
    ├─ Climb_Control_Update   Modules/climb_control.c     气缸
    ├─ Chassis_Control_Update Modules/chassis_control.c   麦轮 + 速度环
    ├─ power_control_run      Modules/power_control.c     功率分配
    └─ 统一 CAN 输出          Modules/can_transmit.c → BSP/bsp_can.c

接收: FDCAN 中断 → BSP/bsp_can.c → Task/can_task.c → Modules/can_receive.c → 各模块
巡检: Task/monitor_task.c → chassis_control / gimbal_link 在线状态
```

`Core/Src/main.c` 现在只剩 HAL 初始化与系统入口（约 243 行，无任何控制逻辑）。