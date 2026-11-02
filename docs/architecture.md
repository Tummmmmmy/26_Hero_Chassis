# RoboMaster 英雄机器人底盘板电控架构说明

本文用于说明 `26_Hero_Chassis` 的软件结构、任务模型、主要数据流以及核心控制模块之间的关系。

该工程属于 RoboMaster 实车项目，代码结构是在实际比赛需求和调试过程中逐步形成的，并非严格按照某一种软件架构模板设计。

硬件侧的器件型号、CAN 接口与通信链路见 [硬件连接与通信拓扑](hardware-topology.png)。

---

## 1. 整体架构

程序可以从四个层次理解：

```text
┌──────────────────────────────────────────────────┐
│                   FreeRTOS Tasks                 │
│  Control / CAN / SuperCap / Monitor / UI / INS   │
└────────────────────────┬─────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────┐
│                     Modules                      │
│  Robot / Chassis / Climb / Power / SuperCap /    │
│        Gimbal Link / CAN / Judge System          │
└────────────────────────┬─────────────────────────┘
                         │
               ┌─────────┴─────────┐
               ▼                   ▼
   ┌────────────────────┐  ┌────────────────────┐
   │        BSP         │  │     Algorithm      │
   │ CAN / SuperCap /   │  │ PID / Ramp /       │
   │ PowerMeter / UART  │  │ Filters / AHRS     │
   └─────────┬──────────┘  └─────────┬──────────┘
             │                       │
             └───────────┬───────────┘
                         ▼
                ┌──────────────────┐
                │     Devices      │
                │ Motor / WS2812 / │
                │      BMI088      │
                └──────────────────┘
```

这里的“层”主要用于理解代码职责，并不意味着模块之间完全没有直接依赖。

例如 CAN 接收、电机反馈和功率控制之间存在较直接的数据传递，这是典型的比赛型嵌入式工程结构。

---

## 2. FreeRTOS 任务模型

系统使用 FreeRTOS 进行任务调度，核心任务如下：

| 任务 | 主要周期 / 触发方式 | 主要职责 |
|---|---|---|
| `All_Init` | 启动阶段 | CAN / 比例阀 / 功率计 / 裁判系统 / 超电 / 底盘参数初始化 |
| `Chassis_Task` | 1 ms | 调用 `Robot_Control_Loop()`，主控制任务 |
| `Can_Decode_Task` | 事件驱动 | 从 CAN 消息队列取帧并路由解析 |
| `SuperCap_Task` | 50 ms | 超电离线检测、充放电决策与 0x601 指令发送 |
| `Monitor_Task` | 10 ms | 底盘电机 / 板间链路 / 功率计在线检测 |
| `UITask` | 10 ms | 裁判系统 UI 绘制（静态 / 动态刷新） |
| `INSTask` | 1 ms | BMI088 读取、温控与姿态解算 |
| `Device_Task` | 低频任务 | WS2812 状态灯 |
| `Print_Entry_Task` | 10 ms | VOFA 波形调试（打印主体当前为注释状态） |
| `Print_Task` | 事件驱动 | ANO 调试协议输出 |

控制相关任务以毫秒级周期运行，而 CAN 接收等通信处理则通过中断 + 消息队列的方式交给任务处理。

---

## 3. 系统启动流程

程序启动后首先完成 MCU 和外设初始化，然后启动 FreeRTOS：

```text
main()
  │
  ├── HAL_Init()
  ├── SystemClock_Config()
  ├── GPIO / DMA / FDCAN / SPI / TIM / UART 初始化
  │
  ├── osKernelInitialize()
  ├── MX_FREERTOS_Init()
  │      ├── 创建任务
  │      ├── 创建消息队列（CAN 接收 / 打印）
  │      └── 创建信号量
  │
  └── osKernelStart()
          │
          ▼
       FreeRTOS
```

`All_Init` 在系统启动后继续完成与 FreeRTOS 相关的初始化：CAN 外设与滤波器配置、比例阀 PWM 启动、功率计、裁判系统串口、超级电容以及底盘参数。

---

## 4. 主控制链路

`Robot_Control_Loop()`（`Modules/robot_control.c`）是应用层的核心控制入口，由 `Chassis_Task` 以 1 ms 周期调用。

每个控制周期主要完成：

1. 每隔 100 个周期向云台板发送一次底盘状态（0x304）
2. 检查失能条件（云台未使能 / 板间链路离线 / 裁判系统切断输出）
3. 处理失能 / 使能切换
4. 上坎状态机与 LED 状态更新
5. 解析速度指令，执行底盘解算与速度环
6. 功率控制分配
7. 统一打包并通过 0x200 发送最终电流

```text
云台指令 0x300 / 0x308
        │
        ▼
  Robot_Control_Loop()（1 ms）
        │
        ├── 100 周期一次：0x304 底盘状态回传
        ├── 失能检查 / 使能切换
        ├── Climb_Control_Update()    气缸状态机 + LED
        ├── Chassis_Control_Update()  麦轮解算 + 速度环
        ├── Power_Control_Update()    功率分配
        └── 0x200 电流指令 → FDCAN2
```

---

## 5. CAN 通信

CAN 是底盘板与云台板、电机和各设备之间的主要通信方式，分为三层：

```text
应用层（chassis / climb / power / supercap）
        ↓
Modules/can_transmit.c   协议打包：DJI 电流 / 双板状态 / 拨盘转发
        ↓
BSP/bsp_can.c            硬件收发：FDCAN 配置、Filter、BSP_CAN_Send、RX 中断
        ↓
HAL_FDCAN
```

### 5.1 接收流程

```text
FDCAN RX Interrupt
        │
        ▼
HAL_FDCAN_RxFifo0Callback()
        │
        ▼
Message Queue（canRxQueueHandle）
        │
        ▼
Can_Decode_Task
        │
        ▼
CAN_Rx_Route()
        ├── 云台指令 / UI 数据（gimbal_link）
        ├── 底盘电机反馈（chassis_control）
        ├── 功率计（bsp_power_meter）
        ├── 超级电容（bsp_SuperCap）
        └── 拨盘转发（can_transmit）
```

中断只负责收帧和投递消息队列，具体解析在 `Can_Decode_Task` 中完成。

按总线划分：

- FDCAN1：0x300 整机指令、0x308 UI 数据、0x303 / 0x301 拨盘转发
- FDCAN2：0x201~0x204 底盘电机反馈、0x301 功率计、0x600 超电
- FDCAN3：0x17 拨盘反馈（转发回云台板）

### 5.2 发送流程

```text
Robot Control
      ├── 0x200 底盘电机电流
      ├── 0x304 底盘状态
      └── 0x302 拨盘反馈
SuperCap Task
      └── 0x601 超电指令
              │
              ▼
        BSP_CAN_Send
              │
              ▼
          FDCAN TX
```

- FDCAN1：0x304 底盘状态、0x302 拨盘反馈
- FDCAN2：0x200 电机电流、0x601 超电指令
- FDCAN3：0x207 / 0x07 拨盘转发

具体 ID 与协议定义以 `can_receive.c`、`can_transmit.c` 以及对应设备驱动为准。

---

## 6. 底盘控制

底盘控制位于 `Modules/chassis_control.c`，负责麦克纳姆轮解算与四轮速度环，模块内部持有四个电机对象与逻辑到物理的 ID 映射。

```text
vx / vy / wz（0x300 指令还原）
        │
        ▼
  麦轮运动学解算
        │
        ▼
  四轮速度环 PID
        │
        ▼
  Chassis_Output_t（目标电流 + 速度误差）
```

- 电机反馈（0x201~0x204）在 `can_receive` 中解码后更新到四个电机对象
- 速度环输出不直接发送，而是先经过功率控制统一分配后再打包为 0x200
- 电流上限默认为 12000，当超级电容处于放电输出状态时提高到 15000
- 逻辑电流与物理电机 ID 的映射由模块本身完成，上层只接触逻辑顺序

---

## 7. 上坎控制

气动上坎位于 `Modules/climb_control.c`，是一个小型状态机，输入为云台指令中的 `climb_mode`。

```text
climb_mode
   │
   ├── 0：全收
   ├── 1：前伸（单次动作）
   └── 2：后伸（带到位检测）
          │
          ▼
     前 / 后气缸 GPIO
```

- 后伸状态会读取到位检测信号，满足条件后才伸出后气缸
- 底盘失能 / 云台链路离线时调用 `Climb_Control_Retract_All()` 前后全部收回
- 当前相位通过 `Climb_Get_Phase()` 提供给控制循环，用于 LED 指示
- 比例阀 PWM 在初始化时启动并保持固定输出

---

## 8. 功率控制

功率控制位于 `Modules/power_control.c`，用于在裁判系统功率限制内尽可能保留底盘动力。

```text
速度环目标电流
      │
      ▼
功率模型预测 P = k1·ω·I + k2·I² + k3·ω²
      │
      ▼
功率预算（上限 + 缓冲能量 PID 调整）
      │
      ▼
超预算时按比例缩放 → 差速分配 → 反解电流
```

- 功率上限：超电放电时按电量档位（>50% / >30% / >10% / 其余）取 390 / 300 / 250 / 150 W；上坎时取 130 W；不使用超电时取裁判系统功率上限
- 缓冲能量 PID：目标 50 J，输出限幅 ±80 W，用于在缓冲能量充足时适度放开功率
- 裁判系统离线时使用 100 W 的保守上限
- 反解电流时区分电流方向（驱动 / 制动），在两个可行解中取能耗更小的一个

---

## 9. 超级电容

超级电容驱动位于 `BSP/bsp_SuperCap.c`（收发与状态），充放电决策位于 `Modules/supercap_control.c`，由 `Task/supercap_task.c` 每 50 ms 调用一次。

- 接收 0x600 状态帧（电量、可用状态等），发送 0x601 控制帧
- 放电条件：云台请求（`supercap_req`）或上坎，且电容可用、电量高于最低输出阈值
- 充电：放电中按裁判功率上限的 0.9 倍充电；非放电状态按缓冲能量（高于 56 J 后随能量线性增加）充电，并做一阶滤波与 120 W 限幅
- 失能或离线时充放电全部禁止
- 离线判定超时 2 s，离线时电流上限回落到保守值

---

## 10. 双板协同

底盘板不直接接收遥控器输入，整机指令均来自云台板。

接收（FDCAN1）：

- `0x300` 整机指令 → `Chassis_Cmd_t`：`vx / vy / wz`、使能模式、上坎、超电请求、LED、摩擦轮状态、键鼠模式等
- `0x308` UI 数据 → `Chassis_UI_t`：云台 Yaw / Pitch 角度与摩擦轮转速

发送：

- `0x304` 底盘状态（约 100 ms 周期）：弹速、热量、热量上限、冷却、颜色、裁判在线
- `0x302` 拨盘反馈转发

链路保护：连续 200 ms 未收到云台数据即判定链路离线，主循环立即失能。

拨盘透传：

```text
FDCAN1 0x303（速度）/ 0x301（模式）
        │
        ▼
    can_receive → can_transmit（Dial_Relay_*）
        │
        ▼
FDCAN3 0x207（速度）/ 0x07（模式）──▶ 达妙拨盘
        │
        ▼
FDCAN3 0x17（拨盘反馈）
        │
        ▼
FDCAN1 0x302 ──▶ 云台板
```

拨盘闭环（校位、射击状态机、卡弹处理）在云台板完成，底盘板只负责总线转发。

---

## 11. 裁判系统

裁判系统数据由 `Modules/RM_JudgeSystem.c` 解析：

```text
USART1
    │
    ▼
JudgeSystem_Callback()
    │
    ├── 帧头 / 长度校验
    ├── CRC16 校验
    └── 按 cmd_id 解析到各数据结构
              │
              ▼
       裁判数据（功率 / 热量 / 状态 / 发射等）
```

- 离线判定超时 1 s，`Judge_IsOnline()` 供其它模块使用
- 底盘侧主要使用：底盘功率上限、缓冲能量、底盘输出开关、发射热量与弹速等
- 云台姿态与底盘状态在 `UITask` 中绘制为裁判系统 UI

---

## 12. 模块职责

### `robot_control`

系统控制入口。

负责协调：

- 失能 / 使能切换
- 上坎
- 底盘
- 功率分配
- CAN 统一发送
- LED 状态

不直接承担底层设备驱动。

### `chassis_control`

负责：

- 麦克纳姆轮解算
- 四轮速度环
- 电机反馈更新
- 逻辑 / 物理电流映射

### `climb_control`

负责气动上坎状态机与气缸输出。

### `power_control`

负责功率模型预测、缓冲能量 PID 与电流分配。

### `supercap_control`

负责超级电容充放电决策（何时充电、何时放电、功率多少）。

### `gimbal_link`

负责与云台板的板间数据契约：指令解析、UI 数据解析、链路在线状态。

### `can_receive`

负责 CAN 接收帧的路由和协议解析入口。

### `can_transmit`

负责各类 CAN 控制指令的打包和发送。

### `RM_JudgeSystem`

负责裁判系统串口协议解析与 UI 绘制。

---

## 13. 状态和数据归属

工程中的主要状态由对应模块维护：

| 状态 | 归属 |
|---|---|
| `Motor_Controller` / `Motor_Feedback` | `Devices/motor.h`（电机通用结构体，各控制模块持有） |
| `CAN_Rx_Message_t` / `canRxQueueHandle` | `BSP/bsp_can.h`（CAN 接收帧结构体与接收队列句柄） |
| 底盘 4 电机、麦轮解算、速度环 PID、逻辑→物理 ID 映射 | `Modules/chassis_control.c`（电机数组为 `static`，对外只提供 `Chassis_Output_t`） |
| 气缸相位 | `Modules/climb_control.c`（`static`，对外只提供 `Climb_Get_Phase()`） |
| 功率控制器（缓冲 PID、功率模型参数、分配结果） | `Modules/power_control.c`（内部 `static`，对外仅 `Init / Update / Get_Currents`） |
| 超级电容收发与状态 | `BSP/bsp_SuperCap.c`（内部 `static` + getter / setter） |
| 超电充放电决策 | `Modules/supercap_control.c`（由 `Task/supercap_task.c` 每 50 ms 调用） |
| 功率计数据 | `BSP/bsp_power_meter.c`（内部 `static` + getter；离线检测由 `Task/monitor_task.c` 巡检） |
| 双板指令 / UI 快照、链路在线状态 | `Modules/gimbal_link.c` |
| 拨盘转发（速度 / 模式 / 反馈） | `Modules/can_transmit.c`（`Dial_Relay_*`） |
| 系统 LED 状态、失能 / 使能切换 | `Modules/robot_control.c` |
| 裁判系统数据 | `Modules/RM_JudgeSystem.c`（各 `ext_*` 数据结构） |

这种组织方式的目的，是尽量让具体模块负责自己的运行状态，而不是由一个全局文件保存所有状态。

---

## 14. 安全机制

### 失能条件

```text
云台未使能（chassis.mode == 0）
            或
板间链路离线（超过 200 ms）
            或
裁判系统切断底盘输出
        │
        ▼
 Chassis_Disable()
 0x200 零电流
 前后气缸全收
 LED = 失能
```

### 离线检测

| 对象 | 超时 | 判定位置 |
|---|---|---|
| 底盘电机 | 反馈超时 | `chassis_control` |
| 板间链路 | 200 ms | `gimbal_link` |
| 超级电容 | 2000 ms | `bsp_SuperCap` |
| 功率计 | 2000 ms | `monitor_task` |
| 裁判系统 | 1000 ms | `RM_JudgeSystem` |

### 上坎安全

- 后伸动作需要到位检测满足条件，避免机构未到位时误动作
- 失能状态下前后气缸强制全部收回

### 功率保护

- 裁判系统离线时功率上限回落到 100 W 的保守值
- 可分配预算设置下限，避免极端情况下出现异常分配


---

## 15. 工程特点

这个项目并不是从零设计的一套通用机器人软件框架，而是在 RoboMaster 实际比赛需求下逐步开发形成的嵌入式控制工程。

它的主要特点是：

- STM32 + FreeRTOS 实时控制
- 多路 CAN 电机通信
- 麦克纳姆轮运动控制
- 功率管理与超级电容
- 气动机构状态机
- 双板协同与整机指令解析
- 裁判系统接入
- 实车调试和参数验证

因此理解该工程时，重点不在于某一个算法文件，而在于：

**输入 → 状态 → 控制 → 通信 → 执行器 → 反馈**

这一整条机器人控制链路。
