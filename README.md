# ROS 三轮移动机器人 STM32 下位机

这是一个用于学习 ROS 移动机器人底层控制的 STM32 下位机工程。

项目面向三轮全向底盘，使用 STM32F103RBT6、Keil MDK、标准外设库和 FreeRTOS，实现了从 ROS 速度指令到电机闭环控制，再到底盘速度反馈的完整链路。

这个仓库只包含 STM32 下位机固件，不包含 ROS 上位机节点。它适合用来理解：ROS 上位机发出的速度指令，最终如何经过串口通信、运动学解算、编码器测速和 PID 控制，变成电机 PWM 输出。

## 可以学到什么

- 三轮全向底盘的正逆运动学解算
- 5 ms 编码器差分采样与 10 ms 滑动窗口测速
- 速度前馈、带 dt 的抗饱和 PID 与三路电机闭环
- 目标速度斜坡、换向关断和轮速同比例限幅
- FreeRTOS 下的周期任务划分
- MPU6050 原始数据采集与上电零偏标定
- STM32 USART + DMA + IDLE 空闲中断接收
- STM32 DMA 非阻塞串口发送
- ROS 指令超时后的自动停车保护
- OLED 状态显示与蓝牙调试控制

## 控制链路

```text
ROS 上位机
   |
   |  USART3: 115200 bps
   v
DMA 环形缓冲区 + IDLE 中断
   |
   v
RosRxTask 解析速度指令
   |
   v
Vx / Vy / W 底盘目标速度
   |
   v
三轮逆运动学解算
   |
   v
三路目标轮速
   |
   v
编码器反馈 -> PID 速度闭环 -> PWM + 方向控制
   |
   v
三路直流减速电机
```

下位机还会反解底盘实际速度，并通过 USART3 回传给 ROS 上位机：

```text
编码器计数 -> 实际轮速 -> 底盘速度 Vx / Vy / W -> RosTxTask -> ROS 上位机
```

## 硬件与开发环境

| 项目 | 说明 |
|---|---|
| MCU | STM32F103RBT6 |
| 底盘 | 三轮全向底盘 |
| 电机 | 3 路直流减速电机，带编码器 |
| 开发工具 | Keil MDK |
| 外设库 | STM32F10x 标准外设库 |
| RTOS | FreeRTOS |
| ROS 通信串口 | USART3，115200 bps |
| 蓝牙调试串口 | USART2，19200 bps |

> 注意：当前 Keil 工程文件 `STM32 工程模版.uvprojx` 中配置的目标器件是 `STM32F103RB`，对应常见芯片型号 `STM32F103RBT6`。如果你的实际开发板使用其他型号，请在 Keil 中核对芯片型号、Flash 容量和启动文件。

## 主要引脚

以下引脚根据当前源码整理。更换开发板或电机驱动板时，需要同步修改对应驱动文件。

| 功能 | STM32 引脚 | 对应代码 |
|---|---|---|
| 电机 PWM 1 / 2 / 3 | PA8 / PA9 / PA10 | `Hardware/pwm.c` |
| 电机 1 方向控制 | PB14 / PB15 | `Hardware/motor.c` |
| 电机 2 方向控制 | PC10 / PC11 | `Hardware/motor.c` |
| 电机 3 方向控制 | PB12 / PB13 | `Hardware/motor.c` |
| 编码器 1 | PA0 / PA1 | `Hardware/encoder.c` |
| 编码器 2 | PA6 / PA7 | `Hardware/encoder.c` |
| 编码器 3 | PB6 / PB7 | `Hardware/encoder.c` |
| ROS 串口 USART3 TX / RX | PB10 / PB11 | `Hardware/USART3.c` |
| 蓝牙串口 USART2 TX / RX | PA2 / PA3 | `Hardware/lanya.c` |
| OLED SCL / SDA | PB8 / PB9 | `Hardware/OLED.c` |
| MPU6050 SCL / SDA | PC0 / PC1 | `Hardware/Mpu6050/MPU6050_I2C.c` |

## FreeRTOS 任务设计

工程将电机控制、ROS 通信、安全检查和显示拆成独立任务，避免 OLED 刷新或串口发送影响 PID 控制周期。

| 任务 | 周期或触发方式 | 作用 |
|---|---:|---|
| `ControlTask` | 5 ms，200 Hz | 读取编码器、计算实际轮速、运动学解算、PID 计算、输出 PWM |
| `RosRxTask` | 串口数据触发 | 从队列中取出 USART3 数据并解析 ROS 指令 |
| `SafetyTask` | 20 ms，50 Hz | 检查 ROS 指令是否超时 |
| `ImuTask` | 4 ms，250 Hz | 读取 MPU6050 原始数据并维护采样状态 |
| `RosTxTask` | 20 ms，50 Hz | 向 ROS 上位机回传底盘实际速度 |
| `DisplayTask` | 100 ms，10 Hz | 在 OLED 上显示轮速和底盘速度 |

当控制源为 ROS 且超过 300 ms 没有收到新指令时，`SafetyTask` 会清零目标速度。随后 `ControlTask` 停止电机并重置 PID，避免恢复通信时积分残留导致电机突然动作。

更详细的 RTOS 改造说明见 [`RTOS_MIGRATION_NOTES.md`](RTOS_MIGRATION_NOTES.md)。本轮控制升级的参数、设计取舍、接线和编译结果见 [`下位机控制改进记录.md`](下位机控制改进记录.md)。

## 目录结构

```text
ros-robot/
├── User/
│   ├── main.c                 # 系统初始化与 FreeRTOS 调度器启动
│   ├── app_tasks.c            # 控制、ROS 通信、安全、显示任务
│   └── FreeRTOSConfig.h       # FreeRTOS 配置
├── Hardware/
│   ├── motor.c                # 三轮运动学、电机方向控制
│   ├── pwm.c                  # TIM1 三路 PWM
│   ├── encoder.c              # TIM2 / TIM3 / TIM4 编码器接口
│   ├── pid.c                  # PID 控制器
│   ├── Mpu6050/               # PC0/PC1 软件 I²C 与 MPU6050 驱动
│   ├── USART3.c               # ROS 串口 DMA 收发
│   ├── my_robot_usart.c       # ROS 通信协议打包与解析
│   ├── lanya.c                # USART2 蓝牙调试控制
│   └── OLED.c                 # OLED 显示
├── Middlewares/
│   └── FreeRTOS-Kernel/       # FreeRTOS 内核
├── library/                   # STM32F10x 标准外设库
├── start/                     # CMSIS 与 STM32 启动文件
├── System/                    # 延时、系统与基础串口代码
├── RTOS_MIGRATION_NOTES.md    # RTOS 化改造记录
├── 下位机控制改进记录.md       # 控制升级参数、保护逻辑与验证记录
└── STM32 工程模版.uvprojx      # Keil 工程文件
```

## ROS 串口协议

ROS 上位机与 STM32 之间使用二进制帧通信。多字节整数按小端序传输，CRC 计算逻辑见 `Hardware/my_robot_usart.c` 中的 `getCrc8()`。

### 上位机下发速度指令

| 字段 | 长度 | 说明 |
|---|---:|---|
| 帧头 | 2 字节 | 固定为 `0x55 0xAA` |
| 数据长度 | 1 字节 | 当前速度指令为 `7` |
| `Vx` | 2 字节 | X 方向速度，`int16_t`，单位 mm/s |
| `Vy` | 2 字节 | Y 方向速度，`int16_t`，单位 mm/s |
| `W` | 2 字节 | 角速度，`int16_t`，实际值放大 1000 倍 |
| 控制位 | 1 字节 | 预留扩展字段 |
| CRC8 | 1 字节 | 对帧头、长度和数据计算 CRC |
| 帧尾 | 2 字节 | 固定为 `0x0D 0x0A` |

### 下位机回传速度反馈

| 字段 | 长度 | 说明 |
|---|---:|---|
| 帧头 | 2 字节 | 固定为 `0x55 0xAA` |
| 数据长度 | 1 字节 | 当前反馈数据为 `9` |
| `Vx` | 2 字节 | X 方向实际速度，单位 mm/s |
| `Vy` | 2 字节 | Y 方向实际速度，单位 mm/s |
| `W` | 2 字节 | 实际角速度，放大 1000 倍 |
| 扩展数据 | 2 字节 | 当前保留为调试字段 |
| 控制位 | 1 字节 | 预留扩展字段 |
| CRC8 | 1 字节 | 对帧头、长度和数据计算 CRC |
| 帧尾 | 2 字节 | 固定为 `0x0D 0x0A` |

USART3 接收使用 DMA 环形缓冲区和 IDLE 空闲中断。中断只负责把新收到的字节送入 FreeRTOS 队列，完整协议解析在 `RosRxTask` 中完成。这样可以减少中断中的工作量，让 200 Hz 电机闭环更稳定。

## 编译与烧录

1. 安装 Keil MDK，并确保已安装 STM32F1 对应的 Device Pack。
2. 使用 Keil 打开 `STM32 工程模版.uvprojx`。
3. 根据实际开发板核对目标芯片型号。
4. 编译工程并烧录到 STM32。
5. 首次运行时架空底盘，先观察轮子方向和编码器反馈是否正确。
6. 连接 ROS 上位机后，从低速指令开始测试。

## 推荐学习顺序

如果你刚开始学习 ROS 移动机器人底层，可以按下面顺序阅读代码：

1. 阅读 `Hardware/motor.c`，理解底盘速度 `Vx / Vy / W` 如何转换为三路目标轮速。
2. 阅读 `Hardware/encoder.c`，理解 STM32 定时器编码器模式如何获得轮子转动计数。
3. 阅读 `Hardware/pid.c`，理解目标轮速、实际轮速和 PWM 输出之间的关系。
4. 阅读 `User/app_tasks.c` 中的 `ControlTask`，串起完整闭环控制流程。
5. 阅读 `Hardware/my_robot_usart.c`，理解通信帧解析、CRC 校验和速度反馈打包。
6. 阅读 `Hardware/USART3.c`，理解 DMA 环形接收、IDLE 中断和非阻塞发送。
7. 阅读 `SafetyTask`，理解为什么移动机器人必须处理通信中断后的停车逻辑。
8. 最后阅读 [`RTOS_MIGRATION_NOTES.md`](RTOS_MIGRATION_NOTES.md)，了解裸机程序改造成 RTOS 架构时的设计取舍。

## 调试建议

- 第一次烧录后先架空轮子，不要直接让机器人落地运行。
- 单独测试每一路电机正反转方向是否正确。
- 手动转动轮子，确认编码器计数方向与电机方向匹配。
- 从较小的 `Vx / Vy / W` 指令开始测试。
- 断开 ROS 串口，确认约 300 ms 后电机自动停止。
- 修改底盘尺寸、电机减速比或编码器线数后，同步调整 `User/app_tasks.c` 中的参数。
- PID 参数需要根据电机、供电和负载重新整定，不建议直接照搬。

## 当前状态与后续方向

当前版本已经完成下位机核心控制链路，适合作为学习和二次开发的起点。后续可以继续扩展：

- 补充 ROS 上位机节点，实现 `cmd_vel` 下发和里程计发布
- 将蓝牙 USART2 接收也改造成队列 + 独立任务
- 将底盘命令封装成结构体，减少全局变量
- 根据实车数据分别标定三路前馈系数并整定 PID
- 确认 IMU 安装坐标系后，将原始数据接入姿态融合
- 增加看门狗、通信丢帧统计和协议帧序号
- 增加接线图、底盘实物图和运行演示

## 说明

本项目主要用于学习 ROS 移动机器人底层控制。不同底盘、电机驱动板和 STM32 开发板的接线与参数可能不同，请结合自己的硬件修改并做好安全测试。
