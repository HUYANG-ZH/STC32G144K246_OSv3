# STC32G144K246 实时架构重构 V1

日期：2026-07-22
状态：V1 第五轮独立审核全部通过；执行代码、交付物与报告已收敛，保留第 6.6 节上板验收项。

## 结论先行

本次重构把“控制、采样、通信、日志”拆成了四种明确的执行域：

1. 与控制时序直接相关的计算只在高优先级定时器中运行；
2. ADC、SPI、UART 的大块数据搬运交给 DMA，IIC 采用一次推进一个硬件命令的后台状态机；
3. 主循环只推进后台事务、协议解析和格式化/日志，不能直接执行电机、负压等实时执行器动作；
4. 所有跨中断/后台的数据以快照、序号或命令邮箱交接，避免半帧数据、重复积分和直接竞争。

因此，原先最明显的控制抖动来源——TIM4 内串行等待 15 次 ADC2 转换、ADC1 电压检测的串行等待、同步 SPI/IIC/UART 事务及后台直接改执行器——已从正常运行路径移除。不能据此虚构具体的微秒级加速比：本工程尚未在目标板上采集 ISR 执行时间和 DMA 吞吐数据。已经证明的是“控制路径不再等待外设完成”，而不是“已经测得某个绝对周期预算”。

参考依据为 [STC32G 技术参考手册](https://stcmicro.com/datasheet/stc32g-cn.pdf)。手册覆盖 ADC/SPI/UART/IIC 的 DMA 与外设控制寄存器；本工程的实现以当前硬件和逐飞库的可验证寄存器接口为边界。

## 1. 工程功能全景

### 1.1 硬件与基础服务

| 功能域 | 当前用途 | 数据/动作所有者 | 实时归属 |
|---|---|---|---|
| 系统时基 | 124 MHz 启动、100 us 时间戳 | `SystemStart`、`service_timetick` | TIM0 |
| 编码器与速度 | 左右轮速度采样 | `service_speed` | TIM3，1 ms |
| 电机 | 左右轮 PWM、休眠/停止 | `service_motor`，由 `app_speedout` 调用 | TIM5，1 ms |
| 负压 | PWM 百分比控制 | `service_negative_pressure` | 请求来自后台/控制，TIM5 实际写 PWM |
| 蜂鸣器 | 启动和告警提示 | `service_buzzer` | 主循环状态推进；无延时等待 |
| 电感 | 5 通道赛道/元素输入 | `bsp_inductor` / `service_inductor` | ADC2 DMA 采样，TIM4，1 ms 消费 |
| 电池 | ADC1 电压检测与欠压保护 | `bsp_battery` / `service_batterycheck` | ADC1 DMA；TIM8，10 ms 保护判定 |
| IMU660RC | 温度、三轴陀螺、三轴加速度 | `bsp_imu` / `service_imu` | SPI3 DMA；TIM7，1 ms 消费 |
| VL53L1X/DL1B ToF | 距离测量 | `bsp_tof_async` / `service_tof` | IIC1(P77/P76) 后台状态机，20 Hz 请求 |
| 无线/调参 | UART、逐飞协议、`wprint` | `zf_driver_uart` / `service_packet` / `service_wireless_uart` | DMA TX / 主循环解析 |
| 调试输出 | `printf`、告警与运行日志 | UART TX 环形缓冲 | 主循环或显式调试入口 |

### 1.2 应用控制功能

| 应用模块 | 职责 | 最终执行位置 |
|---|---|---|
| `app_inductor_preprocess` | 电感快照预处理、归一化与赛道特征 | TIM4 |
| `app_motion_preprocess` | 横向误差和控制前置量 | TIM6 |
| `app_element` | 圆环、气缸、跷跷板等元素识别和状态机 | TIM6；IMU 积分分支在 TIM7 |
| `app_feedforward` | 曲率/前馈计算 | TIM6 |
| `app_speed_plan` | 目标速度规划 | TIM6 |
| `app_motion_postprocess` | 航向角速度闭环、左右轮目标生成 | TIM6；IMU 更新在 TIM7 |
| `app_speedout` | 左右轮速度 PID 和电机/负压执行邮箱 | TIM5 |
| `app_battery_guard` | 原始 ADC 欠压判定与停机请求 | TIM8 |
| `app_boot_sequence` | 启动、停机、延时启动状态机 | TIM2 启动状态机；实际启停仍投递 TIM5 邮箱 |
| `app_scheduler` | 仅保留启动类软调度，不承载闭环控制 | TIM2 触发，主循环推进 |

以下模块仍会被工程编译，但当前 `main.c` 没有初始化或调度它们，故不属于 V1 的运行时控制链：

| 模块 | 当前状态 | 处理结论 |
|---|---|---|
| `app_attitude` | 已编译、未初始化 | 保留为备用姿态实验模块，不读取或控制当前运行链 |
| `app_fuzzy_pid` | 已编译、未初始化 | 保留为算法候选，不在任何 TIM 回调中执行 |
| `app_load_distribution` | 已编译、未初始化 | 保留为备用分配算法，不参与左右轮目标生成 |
| `app_log` | `main.c` 中显式注释初始化 | 不作为日志路径；当前日志走后台 UART 异步队列 |

### 1.3 当前时序表

| 中断 | 周期 | 允许做什么 | 不允许做什么 |
|---|---:|---|---|
| TIM0 | 100 us | 时间戳递增 | 传感器总线、格式化、控制计算 |
| TIM2 | 1 ms | 启动状态机节拍 | 闭环控制、物理等待、直接执行器写入 |
| TIM3 | 1 ms | 编码器速度采样 | 通信/日志 |
| TIM4 | 1 ms | 消费 ADC2 DMA 快照并电感预处理 | 等待 ADC、IIC/SPI/UART |
| TIM5 | 1 ms | 速度 PID、消费启停/负压邮箱并写执行器 | 阻塞通信、日志 |
| TIM6 | 5 ms | 固定顺序的运动/元素/前馈/速度规划/航向计算 | 队列调度、通信、日志 |
| TIM7 | 1 ms | 消费 IMU 快照、IMU 元素积分与超时推进 | SPI 配置、延时、重复处理旧帧 |
| TIM8 | 10 ms | 欠压原始值比较和停机邮箱 | 浮点换算、日志、直接多外设停机 |

主循环的职责严格限于：推进硬件 IIC 异步事务、推进 IMU 启动状态机、发起/收取 ToF、发起电池 DMA 采样、有限预算的函数队列/协议解析、启动软调度、延迟格式化日志和蜂鸣器状态机。它不计算实时闭环，也不直接写运行期电机或负压 PWM。

## 2. 重构前的关键问题与判断理由

| 问题 | 风险 | 本次处理 | 判断理由 |
|---|---|---|---|
| TIM4 内每 1 ms 串行轮询 5 路、每路 3 次 ADC2 | 最坏时间取决于转换完成时间，可能侵占下一个控制周期 | ADC2 扫描 DMA，TIM4 只读平均快照 | 采样等待与控制计算必须解耦 |
| 电池检测串行等待约 20 次 ADC1 | 重新启用保护时会把等待带回主链路 | ADC1 DMA 平均快照，TIM8 只比较原始值 | 保护必须可预测，不能因检测自身阻塞 |
| IMU 读取/初始化和 ToF 使用同步总线接口 | 外设 NACK、慢响应会扩大控制抖动或启动卡顿 | SPI3 DMA 运行帧和异步启动状态机；ToF/IIC 后台状态机 | 控制线程不能等待外设 ACK/IF 位 |
| UART/`printf`/`wprint` 同步发送 | 波特率决定的物理发送时间可远大于控制周期 | UART DMA TX 环形队列；格式化仅留在后台 | “发送”必须立即返回；格式化也不能出现在 ISR |
| 多个控制模块分散定时器/低优先级调度 | 数据先后关系不稳定，难以计算端到端延迟 | TIM6 固定链，TIM5 执行器先于 TIM6 产生下一帧目标 | 明确每一帧的因果顺序才能评估控制稳定性 |
| 后台或包处理直接写执行器 | 低优先级路径可在任意时刻改变 PWM | 命令邮箱，TIM5 统一实际执行 | 执行器所有权必须单一 |
| IMU DMA 帧可能慢于 1 ms 控制节拍 | 同一陀螺数据被多次积分或反复滤波 | 双缓冲快照、有效位、递增序号，TIM7 去重 | “没有新观测”不能伪装成多次新观测 |

## 3. 目标数据流与实际实现

```text
IMU660RC --SPI3 DMA--> 双缓冲完整原始帧 --序号/有效位--> TIM7
                                                  |              |
                                                  |              +--> 元素 IMU 积分/超时
                                                  +--> 温度 + Gyro XYZ + Acc XYZ 服务快照

电感 5 ch --ADC2 DMA--> 平均值快照 ---------------------------> TIM4 预处理
电池 P10 --ADC1 DMA--> 原始平均值快照 -----------------------> TIM8 欠压比较

TIM6: motion_preprocess -> element -> feedforward -> speed_plan -> motion_postprocess
                                                                      |
                                                                      v
TIM5: 速度 PID + 启停/负压邮箱 -> 电机 PWM / 负压 PWM

UART RX/TX DMA, IIC 单步状态机, ToF 状态机, 协议解析, 日志 -> 主循环后台
```

### 3.1 IMU：完整数据而非局部折衷

SPI3 每次事务为 1 字节寄存器地址加 14 字节有效载荷；服务层发布以下完整快照：

- `temperature_raw`；
- `gyro_x/y/z_raw` 与换算后的 `gyro_x/y/z`；
- `acc_x/y/z_raw` 与换算后的 `acc_x/y/z_g`；
- `sequence` 和 `valid`。

这不是只取 `gyro_z` 的控制专用读法。所有七个 16 位物理输出字段都在同一 DMA 帧中读取和发布。运行期以双缓冲发布：DMA 完成回调写入非活动槽，随后原子切换活动槽和序号；TIM7 只消费完整有效快照。`app_motion_postprocess` 和 `app_element` 各自按序号去重，DMA 暂停时仍可让超时/安全状态推进，但不会重复积分或重复更新陀螺低通滤波器。

IMU 启动也改为后台状态机：上电等待、WHO_AM_I、软复位、复位等待、寄存器配置、可选四元数配置、首帧请求依次推进。运行控制不再遇到 SPI 配置或延时。默认关闭可选四元数输出；这不影响温度、三轴陀螺和三轴加速度的完整物理原始帧读取。

为了减少 TIM7 的浮点负担，量程换算的倒数在初始化期计算一次，运行期使用乘法替换每帧六次除法。这一优化不改变标定或单位定义。

### 3.2 ADC：采样后台化、控制消费快照

- 电感：ADC2 配置 5 通道扫描、每通道 8 次采样并形成平均快照。TIM4 不再等待转换完成位。
- 电池：ADC1 对 P10 进行 8 次 DMA 平均。TIM8 只比较原始阈值 `1236`（由 11.0 V 和现有比例换算并向上取整），命中后置位事件并请求 TIM5 停机。
- 电压换算、告警文本和无线日志都由主循环的 `app_battery_guard_pump_events` 执行，而不是 TIM8。

这样，采样频率、DMA 完成时间和控制周期相互隔离；控制周期只承担常数时间的快照复制/计算。

#### 3.2.1 快照发布与传感器失效策略（首轮订正）

ADC1 与 ADC2 都使用两个发布槽。DMA ISR 只写**非活动槽**；所有通道、`sequence` 与 `valid` 都写完后，才在短 `EA=0` 临界区一次切换活动索引。因此 TIM4/TIM8 即便抢占低优先级 DMA ISR，也只能读取上一帧完整快照，绝不会读到“某些通道来自新帧、某些来自旧帧”的混合数据。

- 电池：后台每 100 ms 触发一次 ADC1 DMA；若一次 DMA 忙超过 10 ms，后台仅复位并重装 DMA 寄存器后重试。TIM8 对首帧给出 300 ms 宽限，之后要求新序号间隔不超过 150 ms；低于原始阈值 `1236`、首帧缺失或快照过期都会锁存 `APP_SPEEDOUT_SAFETY_BATTERY`。
- 电感：TIM4 只消费 ADC2 快照。后台在 ADC2 忙超过 10 ms 时重装 DMA；TIM4 对首帧给出 100 ms 宽限，之后新序号最大年龄为 50 ms。过期锁存 `APP_SPEEDOUT_SAFETY_INDUCTOR`；新帧只解除该传感器位，不会自动重新起车。
- IMU：TIM7 按 IMU 序号去重，首帧宽限 100 ms、最大年龄 50 ms。超期锁存 `APP_SPEEDOUT_SAFETY_IMU`；新帧同样只解除该位。

三个安全位由 TIM5 的执行邮箱统一裁决：任一位存在时，`START` 不可覆盖，电机立即归零，负压请求也在**同一 TIM5 周期**执行为 0%。这把“传感器有新数据”与“允许重新启动”明确分开，恢复后仍需要上层显式 `START`。

### 3.3 通信设计

| 通信 | 当前实现 | 为什么不阻塞控制 |
|---|---|---|
| UART TX | 每 UART 512 B xdata 环形队列，DMA TX ISR 接续下一段 | 提交数据后立即返回；满队列记录丢弃，而不等待线路腾空 |
| UART RX | ISR/DMA 接收后写 SPSC 缓冲，主循环有限预算解析 | 接收和解析分离，协议风暴不会直接拖慢控制 ISR |
| `printf` | 输出最终进入 UART 异步队列 | C 格式化本身仍是 CPU 工作，因此禁止在 TIM ISR 中调用；物理发送不等待 |
| `wprint` | 复用异步 UART，加入重入保护和丢弃策略 | 不在控制 ISR 格式化或等待串口 |
| SPI3 | DMA 异步 `spi_dma_async_transfer`，DMA ISR 只发布状态/回调 | IMU 控制读取无 SPI 轮询 |
| 硬件 IIC | `iic_async_transfer` + 一次推进一个命令的状态机 | 主循环每次只检查/下发一个硬件命令，不等待 SCL/SDA/IF |
| ToF | XSHUT、配置写入、ready 轮询、读距均拆成 IIC 后台状态 | 135 B 配置及测距不进入任意控制 ISR |

这里特意没有把 IIC 强行改成“DMA 才算异步”。STC32 硬件 IIC 的事务需要地址、寄存器前缀、Repeated START、最后一字节 NACK 和 STOP 的动态决策；对当前 20 Hz、短事务的 ToF，逐命令状态机已经消除了 CPU 等待，且有更容易验证的错误路径。若未来 IIC 总线承载高吞吐连续块传输，可在不改上层 ToF 接口的前提下替换为 IIC DMA 后端。

旧的同步 SPI/IIC 入口保留为逐飞库兼容层，但当前工程的 IMU、ToF、UART 输出和 ADC 控制路径不调用它们。兼容接口不是实时路径的一部分。`iic_init()` 也已移除“检测到低电平即同步九脉冲恢复”的运行时行为：它只完成常数时间寄存器配置并返回 `IIC_ERROR_BUS_STUCK`，旧 `iic_recover_bus()` 仅为兼容入口。ToF 每次初始或错误退避重试都先走 XSHUT 高—低—高及启动等待，**之后**才调用 `iic_init()`；因此传感器自身占低 SDA/SCL 时仍有机会被异步复位释放。若 `iic_async_transfer()` 连续报告 `IIC_ERROR_BUSY`，ToF 只在后台允许最多 10 ms 的提交等待，随后作为超时进入同一 XSHUT 退避链，避免硬件 BUSY 位永久置位时无限原地提交。NACK、超时、总线被拉低或型号校验失败都会标记无效距离并等待 1 s 后重新进入该流程；错误不会成为永久终态，也不会在控制中断内重试。

## 4. 实时控制与所有权

### 4.1 固定控制链

TIM6 中严格按下列顺序运行：

```text
motion_preprocess
    -> element 状态识别
    -> feedforward
    -> speed_plan
    -> motion_postprocess（生成下一帧左右目标）
```

TIM5 先用上一帧目标完成速度 PID 和实际 PWM 写入；随后 TIM6 计算下一帧目标。这一帧间关系是刻意设计的，而非调度偶然性。TIM7 以 1 ms 频率处理 IMU 新帧和 IMU 相关元素积分；TIM8 只做欠压安全触发。没有紧密控制任务通过主循环函数队列或低优先级包处理执行。

### 4.2 执行器邮箱

`app_speedout` 的 `STOP`、`STOP_ALL`、`START` 请求使用短临界区提交，只有 TIM5 的 `app_speedout_tick` 消费并调用真正的电机/负压操作。负压百分比同样由请求邮箱交给 TIM5，避免下列竞争：包处理、启动状态机、元素状态机或欠压保护在不同上下文直接重写 PWM。

配置包写入带回调的变量时，回调也只更新请求/所有权状态；它不直接触碰运行期 PWM。控制侧设置负压后，会拥有控制覆盖权；操作员显式写入配置时才恢复配置所有权，避免旧配置在下一轮后台任务中覆盖控制请求。

## 5. DSP32 移植与可用性判断

从 `C:\Users\20708\Downloads\STC32_DSP32_HUGE` 移植了 `project/L0_SYS/sys_dpu32.asm`，并加入 `seekfree.uvproj` 和链接输入。该文件将 C251 的 32 位整除运行时助手映射到 STC 的 DPU32：

| C251 运行时符号 | DPU32 操作 |
|---|---|
| `?C?ULDIV?` | 无符号 long 除法 |
| `?C?ULIDIV?` | 无符号 long/int 除法 |
| `?C?SLDIV?` | 有符号 long 除法 |
| `?C?SIDIV?` | 有符号 long/int 除法 |

最新链接映射证明这些符号由 `STC_DPU32` 模块导出：

```text
?C?SIDIV?  00FF0095H
?C?SLDIV?  00FF0091H
?C?ULDIV?  00FF0087H
?C?ULIDIV? 00FF008BH
```

这证明“C251 的 32 位整数除法已经由 DPU32 接管”。它**不是**通用 FIR/FFT/矩阵 DSP 库，也不替代 TFPU 的浮点路径；报告不能把它夸大为已经给所有浮点控制算法加速。对本工程最直接的可用收益是涉及 32 位整数比例/时间计算时不再落到通用软件除法助手。浮点敏感的 IMU 换算已单独通过预计算倒数减少运行期除法。

## 6. 性能、安全性和残余风险

### 6.1 已消除的等待路径

静态扫描当前工程后，`service_delay_ms` 只有定义，没有正常业务调用；`system_delay_ms` 仅存在于 `service_motor_init -> service_motor_reset` 的上电硬件时序（1 ms 高、30 us 低、1 ms 高），没有运行期调用者。同步 `iic_read/iic_write/iic_transfer`、同步 `spi_dma_transfer`、`adc_get` 不在当前工程运行路径中。

这不等于兼容库内的旧同步函数被删除；它们仍可能被未来新代码误用。因此后续编码规范应明确：控制 ISR 与后台实时业务只可使用 `*_async`、DMA 快照或请求邮箱接口。

### 6.2 并发和故障处理

- DMA 采样采用双缓冲或稳定快照；跨上下文复制用短 `EA` 临界区；
- IIC、SPI、UART 的“忙”状态有可查询状态和超时/丢弃策略，而不是无限等待；
- IIC 遇到 NACK、超时或总线异常会结束当前异步事务并报告状态，ToF 保留上一次有效距离/无效状态；
- GPIO 中断分发在调用回调前检查空指针；
- 欠压 ISR 只设置一位事件和停机邮箱，日志在后台执行；
- IMU 未有效、DMA 未产生新序号时，控制不会把旧观测反复积分。

### 6.3 首轮审核后的并发订正

首轮审核发现的四类可复现问题已全部修正：ADC 快照由原位更新改为双缓冲发布；电池保护由低频、无新鲜度判断改为 100 ms 采样、10 ms 恢复与 TIM8 年龄互锁；IMU 与电感均按序号和年龄进入安全抑制；`service_timetick_what()` 读取 32 位 tick 时使用短 `EA=0` 临界区。SPI IMU 完成回调在解析接收缓冲并发布完整帧之前保持传输锁，避免 TIM7 提前复用同一 DMA 缓冲区。

控制目标也改成成对邮箱：TIM6 或包回调只在短临界区发布 `{left,right,sequence,owner}`，TIM5 只在看到新序号时整体消费。`service_packet` 对每个浮点参数写入已使用短临界区；本轮又对读取回复增加完整快照复制，避免后台遥测拼出撕裂的浮点值。PID 参数的逐项更新在协议层是原子写入，双轮速度目标则使用成对邮箱保证帧一致性。

### 6.4 第二轮审核后的通信与交付物订正

第二轮通信审查证明了一个此前未覆盖的故障链：原来的 `bsp_tof_async_init()` 在 XSHUT 配置之前调用 `iic_init()`；总线低电平会使初始化立即返回，导致后续 XSHUT 复位永远不可达。现将 `BSP_TOF_STATE_IIC_INIT` 放到 `XS_HIGH_WAIT -> XS_LOW_WAIT -> XS_BOOT_WAIT` 之后。IIC 失败时仅进入 1 s 后台退避，下一次仍从 XSHUT 复位开始；没有重新引入同步 GPIO 九脉冲或延时循环。

第二轮工具链审查还发现 HEX 落后于最新链接映像。订正后重新编译修改的 `bsp_tof_async.c`、执行全工程对象链接并运行 `OH251`，使 `SEEKFREE`、`SEEKFREE.map` 和 `SEEKFREE.hex` 归属同一交付构建。旧的 `SEEKFREE.build_log.htm` 是 IDE 先前生成的历史文件，不再作为本轮构建证据；准确的命令、产物时间和哈希记录在第 7.1 节。

### 6.5 第三轮审核后的 IIC BUSY 订正

第三轮通信审查发现：`iic_async_transfer()` 在硬件 BUSY 仍置位时返回 `IIC_ERROR_BUSY`，而旧 `bsp_tof_submit()` 将它无限视作普通共享总线忙；ToF 因而可能永久留在任一 `*_SUBMIT` 状态，无法到达 XSHUT 恢复。现增加 `tof_submit_busy_pending` 和 10 ms deadline：第一次 BUSY 允许后台下一轮重试，超期则调用 `bsp_tof_fail(IIC_ERROR_TIMEOUT)`。失败会清除该等待标志、进入 1 s 退避，并在下一轮从 XSHUT 开始；全程没有轮询等待或中断上下文的 IIC 操作。

### 6.6 仍需上板验证的项目

编译和链接无法证明电气时序及实时预算，下列项目必须使用示波器、逻辑分析仪和串口统计完成：

1. SPI3 DMA 的片选、15 B 帧、DMA 完成间隔及 IMU 序号连续性；
2. ADC1/ADC2 DMA 通道顺序、平均值正确性和 ADC ISR 频率；
3. IIC1 P77=SCL、P76=SDA 在 VL53L1X/DL1B 上的 START/Repeated START/ACK/STOP，含 NACK 与断线恢复；
4. UART 负载测试下 TX 丢弃计数、RX 环形缓冲溢出与控制 ISR 抖动；
5. 各 TIM ISR 的高电平包络或 cycle counter 测量，确认最坏时间低于周期；
6. 欠压触发到 TIM5 PWM 归零的最大延迟；
7. 传感器失联、DMA 停滞、IIC NACK、反复启动/停止下的降级行为。
8. C251 **`LARGE` 数据内存模型**下所有 DMA 缓冲区的**物理地址可达性**：最终 `SEEKFREE.map` 明确显示 `MEMORY MODEL: LARGE`；编译选项中的 `ROM(HUGE)` 是代码 ROM 寻址选项，不能把它误称为数据内存 `HUGE` 模型。DMA 地址寄存器只有 16 位，当前链接映射中的缓冲区以逻辑 `xdata` 地址显示。必须在目标板上用 SPI/UART/ADC 三类 DMA 的回读、序号连续性和邻近内存哨兵确认该逻辑地址到芯片物理 xdata 的映射正确；链接成功本身不能证明这一点。若任一项失败，应通过链接定位或专用 DMA 区把缓冲区放到手册允许的物理 xdata 范围内后重新验收。

## 7. 验证记录（V1 实施后）

### 7.1 构建证据

订正构建在 `project/mdk` 以 C251 5.60、L251 4.66.93 和 OH251 1.47 执行。最终 map 的数据内存模型为 `LARGE`；命令中的 `ROM(HUGE)` 仅指定 ROM 寻址范围。链接输入覆盖工程全部对象；链接前重新编译了改变的 `bsp_tof_async.c`，随后检查工程内所有 `.c/.h/.asm/.a51` 均不晚于链接产物，排除“源码比对象更新”的遗漏。

```text
../L1_BSP/bsp_tof_async.c: C251 COMPILATION COMPLETE.  0 WARNING(S), 0 ERROR(S)
Program Size: data=8.6 edata+hdata=8204 xdata=46672 const=18386 code=206354
L251 RUN COMPLETE.  0 WARNING(S), 0 ERROR(S)
OH251: GENERATING INTEL H386 FILE: .\out_file\SEEKFREE.hex
```

当前可烧录交付物是 `project/mdk/out_file/SEEKFREE.hex`（131,420 B，生成时间 2026-07-23 15:27:25，本轮 SHA-256：`30F543865EBC7B665728DB8F9AA1CE7BB765316D7432D0083E74A2E1017634F7`）。其输入映像 `SEEKFREE`（1,238,590 B，15:27:13，SHA-256：`691E7D0EA639E8E9A3F92A407A77789059E41B264003E4B92417D273816FB991`）与 `SEEKFREE.map`（1,604,242 B，15:27:13）来自同一次 L251 运行。HEX 另经独立解析：2,944 条 Intel H386 记录的校验和均正确，且仅有一个 EOF 记录。此前本次新增/重构的 UART、SPI、IIC、ADC1/ADC2、IMU、ToF、TIM5/TIM6/TIM7/TIM8、负压和 ISR 模块也均用相同 C251 选项逐个编译通过。一次早期链接失败来自输出目录仍有旧对象文件（新接口缺符号），在统一重编译对象后消失；该问题是构建产物不同步，非接口或链接脚本设计缺陷。

### 7.2 静态路径核对

已执行的检索结论：

- `pit_ms_init` 只为 TIM2/3/5/6/7/8 分配当前任务，控制闭环不在函数队列；
- 当前 `project` 中无正常运行调用的 `service_delay_ms`、同步 IIC/SPI、`adc_get`；
- `printf`/`wprint` 仍存在于后台、显式 debug 或事件泵函数，未放入 TIM4/5/6/7/8 的实际 ISR 回调；
- `service_motor_reset` 的同步延时仅由 `service_motor_init` 在启动期调用；
- DPU32 映射如第 5 节所示。

`project/mdk/out_file` 是工具生成物，包含 `.obj/.lst/.map/.hex` 等，格式检查会把其中的历史尾随空白当作源代码问题；因此不以该目录的 `git diff --check` 结果评价源代码质量。源码审查应排除生成目录，并在提交前按团队策略决定是否纳入或忽略这些产物。

排除 `project/mdk/out_file` 后执行 `git diff --check` 已通过；该检查覆盖本轮已跟踪源码修改。本文档和其他新增源码将在提交前一并纳入版本控制。

## 8. 审核—订正循环

用户要求每一版使用三名有不同偏好的独立审查者，并在订正后重新三路审核。以下记录将只在实际完成后填写；“通过”必须表示三位审查者都没有阻塞项，而非作者自评。

### 第一轮：已完成，结论为不通过（全部阻塞项已订正）

| 审查角色 | 偏好与覆盖面 | 结论 | 阻塞项 |
|---|---|---|---|
| A：实时/控制审查 | 定时器、最坏时序、ISR、控制所有权、故障降级 | 不通过 | ADC 原位发布可被 TIM 抢占；欠压样本没有新鲜度；IMU/电感停滞未统一降级；32 位 tick 读取非原子 |
| B：通信/并发审查 | UART/SPI/IIC/DMA、状态机、缓冲区、数据竞争 | 不通过 | ADC 快照竞争；IMU DMA 回调过早释放缓冲；32 位 tick 非原子；ToF 错误态缺少退避重试 |
| C：工具链/安全审查 | C251 ABI、DPU32、内存、静态安全、全局架构一致性 | 不通过 | 左右速度目标缺少成对所有权；电池 DMA 可永久停滞；DPU 映射文档过期 |

### 第一轮订正：已完成

1. `bsp_battery`、`bsp_inductor`：由单快照改为双槽发布，活动索引只在帧完成后原子切换；`get_snapshot` 在短临界区复制完整槽。
2. `service_batterycheck`、`service_inductor`：增加 DMA 忙超时、后台无等待恢复和新序号跟踪；`app_battery_guard`、`app_inductor_preprocess`、`app_motion_postprocess` 增加首帧宽限与样本年龄安全互锁。
3. `bsp_imu`：DMA 回调保持传输锁直到 RX 解析与双缓冲发布完成；`service_imu` 继续仅在 TIM7 消费新序号，并把两次校准平均除法改为预计算倒数乘法。
4. `service_timetick`：读取 32 位时间戳使用短 `EA=0` 临界区；所有跨上下文序号/快照读取遵循相同原则。
5. `app_speedout`：新增 `{left,right,sequence,owner}` 目标邮箱和传感器安全位，TIM5 是唯一实际执行者；任一安全位优先于 `START`，且同周期清零负压 PWM。
6. `bsp_tof_async`：错误状态按专用 retry deadline 在 1 s 后重新初始化；静态复核时发现错误态误读普通 deadline，已在本轮额外订正。
7. `service_packet`：写浮点参数本来就在短临界区，本轮增加后台遥测的完整快照复制，避免拼出撕裂的浮点值；格式化与发送仍留在后台。
8. `sys_dpu32` 的映射地址已按最新 `SEEKFREE.map` 更新；仅声明已验证的 32 位整除助手，不夸大为通用 DSP 库。
9. `iic_init()` 不再从活动 ToF 初始化路径进入同步 GPIO 总线恢复；总线低电平以 `IIC_ERROR_BUS_STUCK` 返回，由 ToF 后台退避重试。同步 `iic_recover_bus()` 仍只服务兼容调用者。

订正后，新增/改动的 ToF、ADC、IMU、IIC 驱动、服务、包处理、速度输出模块逐个使用 C251 5.60 编译，均为 `0 WARNING(S), 0 ERROR(S)`；完整链接证据见第 7 节。第一轮审查者均未修改文件，修复由主实现者统一完成，以避免交叉修改引入新的竞争。

### 第二轮：已完成，结论为不通过（阻断项已订正）

| 审查角色 | 偏好与覆盖面 | 结论 | 证据性结论 |
|---|---|---|---|
| A：实时/控制审查 | 定时器、控制所有权、传感器新鲜度、ISR 路径 | 通过 | 未发现控制回调中的同步总线、日志、延时或无界循环；TIM5 仍为运行期电机/负压唯一执行者 |
| B：通信/并发审查 | IIC/ToF、SPI、UART、DMA、状态机 | 不通过 | 总线低电平时 `bsp_tof_async_init()` 先失败于 `iic_init()`，XSHUT 复位不可达，可能无限 ERROR→重试 |
| C：工具链/安全审查 | C251 ABI、DPU32、链接、交付物、内存 | 不通过 | 最新 map/映像与 `SEEKFREE.hex`、IDE 历史 build log 的构建代次不一致，存在烧录旧镜像风险 |

第二轮订正包括：

1. `bsp_tof_async` 新增 `BSP_TOF_STATE_IIC_INIT`，将 IIC 初始化从 `bsp_tof_async_init()` 移到完整 XSHUT 启动等待之后；任何 `BUS_STUCK` 重试都重新经过 XSHUT，不使用同步总线恢复。
2. 使用 C251/L251/OH251 重新生成当前 `SEEKFREE`、map 和 HEX；第 7.1 节记录了可追溯的输出、时间戳和哈希。不得再以旧 `SEEKFREE.build_log.htm` 的时间判断本轮 HEX。

### 第三轮：已完成，结论为不通过（阻断项已订正）

| 审查角色 | 偏好与覆盖面 | 结论 | 证据性结论 |
|---|---|---|---|
| A：实时/控制审查 | 定时器、控制所有权、ISR、传感器新鲜度 | 通过 | 未发现 ToF 修复导致的控制路径回归 |
| B：通信/并发审查 | IIC/ToF、SPI、UART、DMA、状态机 | 不通过 | 硬件 `IIC BUSY` 卡住时，ToF 将 `IIC_ERROR_BUSY` 无界当作普通忙并永久停在提交状态 |
| C：工具链/安全审查 | C251 ABI、DPU32、链接、HEX、DMA 地址 | 通过 | map、链接输入、DPU32 符号与 HEX 校验一致；DMA 地址映射仍为明确的上板验收项 |

第三轮订正：`bsp_tof_submit()` 为 `IIC_ERROR_BUSY` 建立与驱动异步事务同量级的 10 ms 有界等待；超过 deadline 统一走 `IIC_ERROR_TIMEOUT -> ERROR -> 1 s retry -> XSHUT -> iic_init`。该修复同时保持“正常共享总线短暂忙不误判失败”和“硬件 BUSY 永不清除不永久卡死”。

### 第四轮：已完成，结论为不通过（文档阻断项已订正）

| 审查角色 | 偏好与覆盖面 | 结论 | 证据性结论 |
|---|---|---|---|
| A：实时/控制审查 | 定时器、控制、ISR、安全互锁 | 通过 | 未发现 BUSY deadline 导致控制链或安全路径回归 |
| B：通信/并发审查 | ToF/IIC、SPI、UART、DMA | 通过 | 短暂 BUSY 可重试、永久 BUSY 有界失败并回到 XSHUT；未发现新增等待 |
| C：工具链/安全审查 | C251 模型、DPU32、链接、HEX、DMA | 不通过 | 文档将 `ROM(HUGE)` 误写成 C251 数据内存 `HUGE`，而最终 map 显示 `MEMORY MODEL: LARGE` |

第四轮订正：所有 DMA 物理地址验收表述改为 `LARGE` 数据内存模型；`ROM(HUGE)` 单列为代码 ROM 寻址选项。该修正不改变任何可执行代码、链接输入或 HEX，仅纠正报告与实际 map 的语义一致性。

### 第五轮：已完成，结论为通过（三位均无阻断项）

| 审查角色 | 偏好与覆盖面 | 结论 | 核对结果 |
|---|---|---|---|
| A：实时/控制审查 | 定时器、控制所有权、ISR、安全互锁 | 通过 | 未发现最新 ToF 和文档订正对 TIM4–TIM8 或安全链的回归 |
| B：通信/并发审查 | IIC/ToF、SPI、UART、DMA、格式化输出 | 通过 | `IIC BUSY -> timeout -> ERROR -> XSHUT -> iic_init` 闭环有界；每次 IIC 后台推进仍为常数步骤 |
| C：工具链/安全审查 | C251 模型、DPU32、链接、HEX、DMA、文档 | 通过 | map 的 `MEMORY MODEL: LARGE`、`ROM(HUGE)` 语义、DPU32 符号、链接输入、HEX 校验与报告一致 |

因此 V1 的审核—订正循环在第五轮收敛。该结论只覆盖源码、构建和静态路径；第 6.6 节列出的电气时序、DMA 物理地址映射与最坏 ISR 时间仍必须上板验收。

## 9. 后续建议

V1 的首要目标是切断等待和澄清所有权，已经完成。下一阶段应优先做板级测量而不是继续猜测优化：先给每个关键 TIM ISR 加测试 GPIO，测出最大/平均占空和调度抖动；再以实际值决定是否需要进一步把 IIC 换为 DMA、降低日志频率、调整 DMA 缓冲深度或重定控制周期。任何新的传感器或协议接入都应复用“DMA/异步事务 -> 快照 -> 定时器消费”的结构。
