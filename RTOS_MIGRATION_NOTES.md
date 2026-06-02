# STM32 ROS 底盘下位机 RTOS 化改造记录

## 1. 改造目标

原工程是裸机结构，主循环同时负责电机闭环、OLED 显示、ROS 回传、延时等待等逻辑。这样在功能少的时候可以跑，但问题是控制周期不稳定：

- `main while(1)` 中有 OLED 刷新和串口发送。
- `usartSendData()` 后面有 `delay_ms(13)`，会直接阻塞 PID 控制。
- TIM8 中断负责测速，主循环负责 PID，测速和输出不在同一个固定周期内。
- USART3 中断里直接解析完整 ROS 协议帧，中断逻辑偏重。

本次改造的目标是把系统拆成 FreeRTOS 任务，让底盘控制链路稳定、通信链路解耦、显示和调试逻辑不影响电机闭环。

核心原则：

```text
电机控制固定周期、高优先级
通信只更新目标速度
显示只读取状态
安全逻辑负责超时停车
```

## 2. 新增文件

### `User/FreeRTOSConfig.h`

新增 FreeRTOS 配置文件，主要配置：

- 1 ms 系统 tick。
- 6 个任务优先级。
- 12 KB FreeRTOS heap。
- Cortex-M3 的 SVC、PendSV、SysTick 中断入口映射。
- NVIC 使用 4 个优先级 bit。

关键配置：

```c
#define configTICK_RATE_HZ  (( TickType_t ) 1000 )
#define configMAX_PRIORITIES 6
#define configTOTAL_HEAP_SIZE ( ( size_t ) ( 12 * 1024 ) )

#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
```

### `User/app_tasks.h`

新增任务层头文件，提供：

- `App_CreateTasks()`：创建所有 RTOS 任务。
- `App_RosRxByteFromISR()`：兼容单字节接收入口。
- `App_RosRxBufferFromISR()`：USART3 DMA/IDLE 收到一段新数据后调用，把字节批量送入队列。

### `User/app_tasks.c`

新增 RTOS 任务实现，是本次改造的核心文件。

当前任务划分如下：

| 任务 | 周期 / 触发 | 优先级 | 职责 |
|---|---:|---:|---|
| `ControlTask` | 5 ms / 200 Hz | 最高 | 读编码器、计算轮速、底盘速度反解、PID、电机 PWM 输出 |
| `RosRxTask` | 串口字节触发 | 高 | 解析 ROS 下发速度指令 |
| `SafetyTask` | 20 ms / 50 Hz | 高 | 检查 ROS 指令超时，超时后停车 |
| `RosTxTask` | 20 ms / 50 Hz | 中 | 向 ROS 回传底盘实际速度 |
| `DisplayTask` | 100 ms / 10 Hz | 低 | OLED 显示轮速和底盘速度 |

## 3. `main.c` 的改动

原来的 `main.c` 中包含大量业务逻辑：

- 蓝牙控制判断。
- 三轮运动学解算。
- PID 计算。
- 电机 PWM 输出。
- OLED 显示。
- ROS 串口回传。
- `delay_ms(13)`。

现在 `main.c` 只保留系统初始化和任务启动：

```c
int main(void)
{
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    OLED_Init();
    motor_Init();
    USART2_Config();

    PID_Init(&mypid1, Kp, Ki, Kd, Iout, Out);
    PID_Init(&mypid2, Kp, Ki, Kd, Iout, Out);
    PID_Init(&mypid3, Kp, Ki, Kd, Iout, Out);

    App_CreateTasks();
    USART3_Config();

    vTaskStartScheduler();

    while (1)
    {
        motor_Set1(0);
        motor_Set2(0);
        motor_Set3(0);
    }
}
```

这样改的原因：

- `main.c` 不再承担业务调度，业务由 FreeRTOS 调度。
- 控制周期不再被 OLED、串口发送、延时函数影响。
- 初始化流程更清楚，后续维护时更容易定位模块边界。

## 4. 电机测速与闭环控制改动

### 原裸机逻辑

原来测速逻辑在 `Hardware/timer.c`：

```text
TIM8 10 kHz 中断
每 50 次读取一次 TIM2/TIM3/TIM4 编码器
等效 5 ms 采样一次速度
```

PID 控制则在 `main while(1)` 里执行。

问题是测速和 PID 输出不在同一个明确的周期里。主循环中还夹杂 OLED、串口和 delay，导致 PID 调用周期不稳定。

### RTOS 后逻辑

现在 `ControlTask` 每 5 ms 执行一次：

```text
读取 TIM2/TIM3/TIM4 编码器计数
换算三个轮子的实际速度
反解底盘实际速度 Vx_cal / Vy_cal / W_cal
读取最新目标速度 Vx_dipan / Vy_dipan / W
三轮逆运动学解算目标轮速
三路 PID 计算
输出 PWM
```

这样做的原因：

- 编码器采样、速度计算、PID、电机输出属于同一条闭环链路。
- 它们应该在同一个固定周期内完成，避免使用旧速度或旧目标。
- `ControlTask` 独占电机 PWM 输出，避免多个任务同时写电机造成竞态。

### 为什么周期仍然是 5 ms

继续使用 5 ms 是为了沿用原工程的有效控制尺度：

```text
原 TIM8 配置：10 kHz
原逻辑：num == 50 时测速
50 / 10000 = 0.005 s = 5 ms
```

5 ms 对这个三轮底盘比较合适：

- 比 ROS 指令频率更快，下位机可以稳定执行上位机目标速度。
- 比 1 ms 更不容易被编码器低速计数噪声影响。
- 比 20 ms 响应更快，低速和转向控制更平顺。
- STM32F103 跑 200 Hz 速度环有足够余量。

## 5. ROS 串口接收改动

### 原逻辑

原来 USART3 中断直接调用：

```c
usartReceiveOneData(&Vx_receive, &Vy_receive, &W_receive, &testRece4);
```

这意味着中断里要维护协议状态机，还会在解析成功后直接修改底盘目标速度。

### 新逻辑

现在 USART3 使用 DMA 环形缓冲接收，USART3 中断只处理 IDLE 空闲事件：

```c
void USART3_IRQHandler(void)
{
    USART3_DMARxIdleHandlerFromISR();
}
```

`USART3_DMARxIdleHandlerFromISR()` 会根据 `DMA1_Channel3` 的当前写入位置，计算本次新增字节范围，并调用：

```c
App_RosRxBufferFromISR(buffer, length);
```

字节批量进入 FreeRTOS queue 后，由 `RosRxTask` 调用协议解析函数：

```c
usartParseOneByte(byte, &vxReceive, &vyReceive, &wzReceive, &ctrlFlag)
```

这样改的原因：

- 中断变短，系统实时性更好。
- 协议解析从硬件 USART 中解耦，DMA只负责搬运字节，任务只负责解析协议。
- ROS 通信异常不会卡住电机控制任务。

## 6. 协议层改动

在 `Hardware/my_robot_usart.c` 中新增：

```c
int usartParseOneByte(
    unsigned char data,
    int *p_x_SpeedSet,
    int *p_y_SpeedSet,
    int *p_w_SpeedSet,
    unsigned char *p_crtlFlag
);
```

这个函数只负责吃一个字节并推进协议状态机。

原来的 `usartReceiveOneData()` 仍然保留，但它现在只是：

```c
unsigned char data = USART_ReceiveData(USART3);
return usartParseOneByte(data, ...);
```

保留旧函数的原因：

- 降低改造风险。
- 如果某些旧调试代码还调用 `usartReceiveOneData()`，不会直接失效。
- 新任务层使用 `usartParseOneByte()`，逐步完成解耦。

## 7. ROS 回传改动

原来 ROS 回传在 `main while(1)` 中执行：

```c
usartSendData(Vx_cal, Vy_cal, W_cal * 1000, testSend4, testSend5);
delay_ms(13);
```

问题是 `delay_ms(13)` 会阻塞主循环，直接影响 PID 调用周期。

现在改为 `RosTxTask`，并通过 `USART3_SendBytesDMA()` 非阻塞发送：

```text
每 20 ms 回传一次
等效 50 Hz
```

原因：

- ROS 端通常不需要 200 Hz 的底盘状态回传。
- 50 Hz 对里程计和速度反馈已经足够。
- 回传频率与底盘控制频率解耦，DMA启动后立即返回，串口硬件发送不会拖慢电机控制。

## 8. OLED 显示改动

原来 OLED 显示也在主循环中执行。

现在改为 `DisplayTask`：

```text
每 100 ms 刷新一次
等效 10 Hz
```

原因：

- OLED 刷新慢，不应该和电机控制放在同一个周期。
- 人眼看 10 Hz 的状态显示已经足够。
- 显示异常或变慢不会影响底盘闭环。

## 9. 安全任务改动

新增 `SafetyTask`：

```text
每 20 ms 检查一次
如果当前控制源是 ROS，并且超过 300 ms 没收到新速度指令，则停车
```

停车时会把目标速度清零：

```c
Vx_dipan = 0.0f;
Vy_dipan = 0.0f;
W = 0.0f;
```

实际电机停车动作仍由 `ControlTask` 执行。

这样设计的原因：

- 安全逻辑可以强制目标为 0。
- 电机 PWM 输出仍只有 `ControlTask` 一个 owner。
- 避免多个任务同时写电机造成输出混乱。

## 10. 蓝牙控制处理

蓝牙 USART2 解析逻辑暂时保留原方式，但中断优先级已经下调：

```c
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
```

`ControlTask` 会读取蓝牙解析后的 `flag/f/x/w/flag_stop`：

- `flag == 1`：更新目标速度。
- `flag_stop == 1`：停车。

这属于过渡方案。

后续更完整的工程化方案是把蓝牙也改成：

```text
USART2_IRQHandler 只收字节
BluetoothTask 解析协议
统一更新 ChassisCmd
```

## 11. PID 改动

新增：

```c
void PID_Reset(PID *pid);
```

用于停车和超时时清除：

- 当前误差。
- 上一次误差。
- 积分项。
- 输出值。

原因：

- 急停或通信超时后，积分项如果不清零，恢复运动时可能出现电机突然冲一下。
- RTOS 中控制任务周期更稳定，PID 状态也应该明确管理。

## 12. 中断和 FreeRTOS 适配

### NVIC 分组

`main.c` 中改为：

```c
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
```

原因：

- FreeRTOS Cortex-M 移植层推荐所有优先级位用于抢占优先级。
- 便于 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 正确工作。

### USART3 优先级

USART3 中断会调用 `xQueueSendFromISR()`，所以不能是最高优先级。当前设置：

```c
NVIC_IRQChannelPreemptionPriority = 6;
```

### FreeRTOS 异常入口

`stm32f10x_it.c` 中移除了模板里的空实现：

- `SVC_Handler`
- `PendSV_Handler`
- `SysTick_Handler`

这些入口由 FreeRTOS portable 层提供。

## 13. Keil 工程文件改动

`STM32 工程模版.uvprojx` 中新增 include path：

```text
.\Middlewares\FreeRTOS-Kernel\include
.\Middlewares\FreeRTOS-Kernel\portable\RVDS\ARM_CM3
```

新增 FreeRTOS 源文件：

```text
Middlewares\FreeRTOS-Kernel\tasks.c
Middlewares\FreeRTOS-Kernel\list.c
Middlewares\FreeRTOS-Kernel\queue.c
Middlewares\FreeRTOS-Kernel\portable\RVDS\ARM_CM3\port.c
Middlewares\FreeRTOS-Kernel\portable\MemMang\heap_4.c
```

## 14. 当前验证情况

本机尝试执行 Keil 命令行完整构建时，仍然被原工程的编译器配置问题挡住：

```text
Using Compiler 'V6.22'
ARMCLANG\Bin\ArmCC CreateProcess failed
```

这个问题发生在源码真正编译之前，不是 RTOS 代码错误。

为了做代码层面的检查，已经使用 ARMCC 5 对以下文件单独编译通过：

```text
User/main.c
User/app_tasks.c
User/stm32f10x_it.c
Hardware/my_robot_usart.c
Hardware/pid.c
Middlewares/FreeRTOS-Kernel/tasks.c
Middlewares/FreeRTOS-Kernel/queue.c
Middlewares/FreeRTOS-Kernel/list.c
Middlewares/FreeRTOS-Kernel/portable/RVDS/ARM_CM3/port.c
Middlewares/FreeRTOS-Kernel/portable/MemMang/heap_4.c
```

## 15. 后续调试建议

首次上电不要直接让车落地跑，建议架空轮子测试。

建议顺序：

1. 先解决 Keil 编译器配置，保证工程能完整 Rebuild。
2. 上电后确认系统不 HardFault。
3. 不发送 ROS 速度时，电机应保持停止。
4. 发送 ROS 速度指令，观察三个轮子是否响应。
5. 拔掉 ROS 通信，确认约 300 ms 后自动停车。
6. 测试蓝牙 `z/y` 急停和恢复。
7. 观察 OLED 是否 100 ms 左右刷新一次。
8. 架空测试稳定后再落地低速测试。

## 16. 后续可以继续优化的方向

当前版本是“核心 RTOS 化”，不是最终形态。后续可以继续做：

- 把蓝牙 USART2 也改成队列 + `BluetoothTask`。
- 把 `Vx_dipan/Vy_dipan/W` 封装成 `ChassisCmd` 结构体，减少全局变量。
- 给 PID 增加 `dt` 参数，让 Ki/Kd 与控制周期解耦。
- USART3 发送已经改成 DMA，后续可以增加发送队列和丢帧统计。
- IMU 单独做 `ImuTask`，不要放入 `ControlTask`。
- 增加看门狗任务，防止调度器或控制任务卡死。

## 17. USART3 DMA + IDLE 通信改造说明

这次优先处理 USART3，因为它是 ROS 上位机和下位机之间的主通信链路。底盘项目里，串口不是单纯“能收能发”就够了，还要保证通信不能影响电机闭环周期。

原来的问题：

- 接收侧每来 1 个字节就进一次 `USART3_IRQHandler()`，中断频繁。
- 中断里直接推进 ROS 协议解析，业务逻辑偏重。
- 发送侧 `USART_Send_String()` 逐字节等待 `TXE`，会阻塞 `RosTxTask`。
- 如果后续增加更多状态回传或调试数据，阻塞发送会放大控制周期抖动。

现在的实现：

```text
USART3_RX -> DMA1_Channel3 -> 环形接收缓冲区
USART3 IDLE 中断 -> 判断本次新增字节范围 -> App_RosRxBufferFromISR()
RosRxTask -> usartParseOneByte() -> 更新 Vx/Vy/W 目标速度

RosTxTask -> usartSendData() 打包反馈帧
USART3_TX <- DMA1_Channel2 <- 发送缓冲区
DMA1_Channel2_IRQHandler -> 发送完成后释放 busy 标志
```

这样做的作用：

- DMA负责搬运字节，CPU不再逐字节读写串口寄存器。
- IDLE中断只负责判断一段数据到达，不在中断里做完整业务解析。
- 协议解析仍在 `RosRxTask` 中完成，符合 RTOS 分层。
- 发送启动 DMA 后立即返回，`RosTxTask` 不再因为串口硬件发送时间阻塞。
- 如果上一帧还没发完，`usartSendData()` 返回 `0`，当前帧可以丢弃；底盘反馈是实时状态，优先保证控制实时性。

### DMA 原理

DMA 是 Direct Memory Access，直接存储器访问。它的核心作用是让外设和内存之间直接搬运数据，CPU只负责配置，不负责每个字节的复制。

没有 DMA 时：

```text
接收：USART3->DR -> CPU读取 -> CPU写入RAM
发送：RAM -> CPU读取 -> CPU写入USART3->DR
```

使用 DMA 后：

```text
接收：USART3->DR -> DMA硬件 -> RAM缓冲区
发送：RAM缓冲区 -> DMA硬件 -> USART3->DR
```

DMA需要配置的关键参数：

- 外设地址：这里是 `&USART3->DR`。
- 内存地址：接收缓冲区或发送缓冲区。
- 方向：接收是外设到内存，发送是内存到外设。
- 长度：接收环形缓冲区长度，或发送帧长度。
- 地址递增：USART数据寄存器不递增，RAM缓冲区递增。
- 模式：接收用循环模式，发送用普通模式。
- 中断：发送完成后需要中断通知软件释放 busy 标志。

STM32F103 上 USART3 对应的 DMA 通道：

```text
USART3_TX -> DMA1_Channel2
USART3_RX -> DMA1_Channel3
```

### 为什么接收用 DMA 环形缓冲

ROS 串口接收不适合假设“一次刚好来一帧”。实际可能出现：

- 一次只收到半帧。
- 两帧连续到达，形成粘包。
- 偶发丢字节，导致 CRC 或帧尾校验失败。

所以接收侧使用环形缓冲：

```text
DMA持续写 rx_buffer[0..127]
写到末尾自动回到0
软件记录上次处理位置 s_usart3RxLastPos
IDLE中断时计算DMA当前写入位置
把新增区间送入 FreeRTOS 队列
```

协议层仍然逐字节解析，所以即使半包、粘包、错误帧出现，也可以靠帧头 `0x55 0xaa` 和 CRC 重新同步。

### IDLE 中断是什么

IDLE 是 USART 的“接收线空闲”标志。串口已经接收过数据后，如果总线保持空闲超过约 1 个字符时间，硬件会置位 IDLE。

115200bps、8N1 下，一个字符约 10 bit：

```text
10 / 115200 ≈ 86.8 us
```

IDLE 的意义不是“收到一个完整协议帧”，而是“刚刚那一段连续数据暂时结束了”。它特别适合配合 DMA：

- DMA负责持续接收字节。
- IDLE负责提醒CPU处理刚收到的一段数据。

清除 IDLE 标志必须按 STM32 要求先读 `SR` 再读 `DR`：

```c
dummy = USART3->SR;
dummy = USART3->DR;
```

这里读 `DR` 只是为了清中断标志，业务数据已经被 DMA 搬到了 RAM 缓冲区。

### 为什么发送用 DMA 普通模式

下位机反馈帧长度固定，目前是 15 字节，发送前已经知道长度，所以不需要环形 DMA。

发送流程：

```text
usartSendData() 打包15字节协议帧
USART3_SendBytesDMA() 复制到静态发送缓冲区
配置 DMA1_Channel2 传输长度
启动 USART3 DMA TX 请求
DMA自动把字节写入 USART3->DR
发送完成后 DMA1_Channel2_IRQHandler 清 busy
```

115200bps 下，15 字节发送时间约为：

```text
15 * 10 / 115200 ≈ 1.3 ms
```

如果任务阻塞等待这 1.3 ms，50 Hz 回传看似还能接受，但它会和控制、显示、安全任务争时间。改成 DMA 后，CPU只需要很短时间启动发送，真正的串口移位由硬件完成。

### 后续建议

当前版本已经解决“阻塞发送”和“逐字节接收中断”两个核心问题。后续如果要继续工程化，可以加：

- `RosTxTask` 统计 `usartSendData()` 返回 `0` 的次数，用来判断串口是否过载。
- 增加 TX 软件队列，重要状态帧可以排队发送，普通反馈帧可以覆盖旧帧。
- 给 ROS 协议增加帧序号，方便上位机判断丢包和延迟。
- 把 `Vx_dipan/Vy_dipan/W` 封装成结构体，并用 mutex 或临界区统一访问。

## 18. `TIM8_UP_IRQn` 编译报错处理

报错：

```text
Hardware\timer.c(59): error: #20: identifier "TIM8_UP_IRQn" is undefined
```

原因是当前 Keil 工程目标芯片是 `STM32F103RB`，属于 medium-density 器件；标准库在该器件类型下不会定义 `TIM8_UP_IRQn`。而 `Hardware/timer.c` 是旧裸机版本的 TIM8 测速代码，虽然 RTOS 版本已经不再调用 `Timer_Init()`，但该文件仍在工程中参与编译。

处理方式：

- 保留旧 TIM8 代码，但只在 `STM32F10X_HD` 或 `STM32F10X_XL` 下编译。
- 对当前 `STM32F103RB` 目标，提供一个空的 `Timer_Init()`。
- RTOS 版本测速已经由 `ControlTask` 每 5 ms 读取 TIM2/TIM3/TIM4 编码器完成，因此禁用旧 TIM8 定时器不影响当前控制链路。
