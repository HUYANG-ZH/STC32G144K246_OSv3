# STC32G144K246 车模工程：性能优先架构重构报告

## 1. 结论与边界

本次重构把**控制闭环、传感器采样、物理执行、通信与诊断**拆成了不同的实时域：

```text
DMA/外设完成 ──> 原子快照 ──> TIM 控制步骤 ──> 执行请求邮箱 ──> TIM5 写 PWM/电机
       │                                    │
       └──────── 后台状态机 / 报文 / 日志 ──┘
```

结果如下。

| 目标 | 实现结果 | 作出该设计的原因 |
| --- | --- | --- |
| IMU 全量读取 | SPI3 DMA 每帧读取 `OUT_TEMP_L` 起连续 14 个数据字节：温度、陀螺仪 XYZ、加速度 XYZ | 不能以省时为理由牺牲状态观测；DMA 把总线等待从 CPU 控制路径移走 |
| UART / `printf` / `wprint` | 所有 UART TX 走 DMA 环形队列；格式化只允许后台执行 | 控制中断不等待波特率，也不承担格式化开销 |
| 硬件 IIC | IIC1 CH3，P77=SCL、P76=SDA，后台单步状态机，10 ms 真实时间超时 | VL53L1X/DL1B 的总线等待不能挤占控制时隙；此前已实测读到 `fw=0x03, model=0xEA` |
| SPI | 新的异步 SPI DMA API，SPI3 服务 IMU；丢失 DMA 完成中断可超时中止并重启 | 防止一个异常传输永久占用 IMU 通道 |
| 控制任务 | 速度、电感、速度输出、循迹/元素/前馈/速度规划、IMU 滤波、低压停机均在 TIM ISR 中 | 实时性不能依赖后台队列轮询的不可预测时延 |
| ADC | 电感 ADC2 和电池 ADC1 均改为硬件扫描 + DMA + 快照 | 删除控制定时器中的逐通道转换等待 |
| DSP | 迁入 STC DPU32 的 C251 32 位除法运行时实现；原有 TFPU 继续承担浮点运算 | 供体工程实际只包含 DPU32 除法入口，未虚构不存在的 FFT/FIR 库 |

本报告的“通过”仅表示：源码审查、C251 编译和 L251 链接通过。示波器时序、整车闭环稳定性和传感器故障注入必须按第 12 节在目标板上完成后，才能视为硬件性能验收完成。

## 2. 分析方法与证据链

重构不是先替换接口再解释，而是按下面顺序进行。

1. 从 `main.c`、ISR 向下追踪实际运行路径，区分“已编译的兼容 API”和“会在车辆运行时被调用的 API”。
2. 枚举所有定时器、DMA 通道、队列、报文回调、延时和轮询循环，找出可能阻塞或产生不可预测抖动的点。
3. 先建立非阻塞的外设完成模型（DMA/状态机/快照），再把紧密闭环搬入定时器。
4. 用临界区只保护发布指针或快照，不在临界区做长传输、格式化或计算。
5. 编译、链接、检查 map 中的 DPU32 符号；随后按第 13 节进行独立审查—订正—再次审查。

因此，性能结论来自可追踪的数据流和已链接的对象，而不是只根据“使用了 DMA”作推断。

## 3. 工程功能全量盘点

### 3.1 分层职责

| 层 | 当前功能 | 主要文件 / 说明 |
| --- | --- | --- |
| L0_SYS | 启动、时钟、TFPU、DPU32 C251 运行时 | `project/L0_SYS/sys_start.*`、`sys_tfpu.asm`、新增 `sys_dpu32.asm` |
| L1_BSP | 电池、蜂鸣器、编码器、IMU、五路电感、双电机、DL1B ToF | `project/L1_BSP/bsp_*.c`；采样类 BSP 只做外设与快照 |
| L2_SERVICE | 时基、包队列、无线串口、命令协议、电池/IMU/电感/ToF 服务、电机与负压意图 | `project/L2_SERVICE/service_*.c`；不承担高实时控制计算 |
| L3_APP | 电感预处理、运动预/后处理、前馈、速度规划、元素识别、速度输出、启动流程、安全保护 | `project/L3_APP/app_*.c`；所有闭环关键步骤由对应 TIM 触发 |
| LL_Shared_Source | LPF、PID、位置 PID 等可复用数学模块 | `project/LL_Shared_Source` |
| libraries | STC32 外设驱动、逐飞设备兼容层、公共定义 | `libraries/zf_driver`、`libraries/zf_device` |

### 3.2 已链接但当前未进入运行时链路的功能

“存在源文件”不等于“正在影响车辆时序”。以下模块已逐一检查，并在本版本明确标注状态：

| 模块 | 功能 | 当前状态 | 架构结论 |
| --- | --- | --- | --- |
| `app_attitude` | 对 `gyro_z` 的独立低通姿态快照 | 未在 `main.c` 初始化或调度 | 保留为可选算法，不能假设它参与当前控制 |
| `app_fuzzy_pid` | 7×7 双线性模糊 PID 参数修正 | 未被当前闭环调用 | 计算量较高；启用前必须放入明确的 TIM 控制预算中 |
| `app_load_distribution` | 横向载荷转移及左右差速分配 | 未被当前闭环调用 | 启用前应合入 TIM6 控制链，而非后台任务 |
| `app_log` | IAP 页擦写/掉电日志 | `main.c` 中 `app_log_init()` 保持注释 | IAP 擦写可能耗时，禁止直接在控制路径启用 |
| 相机、屏幕、BLE/WiFi 等逐飞设备库 | 通用外设兼容实现 | 被工程链接但 `main.c` 未初始化为当前车功能 | 不计入当前运行时 WCET；新增启用时必须重新做调度审查 |
| 同步 IIC/SPI/软件 IIC/同步 ADC | 历史兼容接口 | 库中保留，运行时链路不调用 | 已隔离，详见第 6.4 节 |

### 3.3 上电与后台功能

`project/user/main.c` 的初始化顺序为：时基、后台队列、调度器、无线/报文、电池、蜂鸣器、IMU、ToF、电机、负压、速度、电感、运动处理、元素、速度输出、安全保护、启动流程。

主循环只做以下非实时工作：

- 推进硬件 IIC 事务状态机；
- 推进 IMU 与 ToF 的启动/配置状态机；
- 消费电池 DMA 快照；
- 有上界地解析报文和兼容延迟队列；
- 运行启动状态机和事件日志泵；
- 推进蜂鸣器和负压的非实时状态。

后台**不直接执行**速度 PID、循迹/元素计算、前馈、速度规划、PWM 或电机写入。启动状态机和无线命令只发布请求；TIM5 统一把请求落到电机/PWM 寄存器。

### 3.3 当前实时定时表

| 中断 / 周期 | 任务 | 优先级 | 数据输入 → 输出 | 设计理由 |
| --- | --- | ---: | --- | --- |
| TIM0 / 100 µs | 单调时间基 | 默认 | 全局 tick | 为非阻塞状态机提供真实时间，不做控制计算 |
| TIM2 / 1 ms | 仅调度器 ready 标志 | 默认 | 后台任务就绪 | 不承载任何闭环动作 |
| TIM3 / 1 ms | 编码器采样、低通、速度快照 | 3 | 编码器 → `service_speed` 快照 | 速度反馈的固定采样时间最重要 |
| TIM4 / 1 ms | 电感快照、7 点去极值平均、归一化、下一次 ADC2 DMA 请求 | 3 | ADC2 快照 → 电感特征 | 替代串行 ADC 等待，持续提供循迹输入 |
| TIM5 / 1 ms | 速度输出 PID、执行请求、负压实际 PWM | 3 | 目标/反馈/邮箱 → 电机 PWM | 唯一物理驱动落点，输出时序可预测 |
| TIM6 / 5 ms | 电感运动预处理、元素控制、前馈、速度规划、后处理 | 3 | 电感/元素 → 左右速度目标 | 控制链固定顺序，禁止后台队列介入 |
| TIM7 / 1 ms | 消费完整 IMU 快照、滤波、元素 IMU 步骤、发起下一帧 | 3 | 温度/陀螺/加速度 → 姿态/元素输入 | IMU 接口使用 DMA，控制仍按固定 1 ms 节拍消费 |
| TIM8 / 10 ms | 原始电压阈值安全保护 | 3 | 电池快照 → STOP_ALL 邮箱 | 低压保护可在下一次 TIM5 输出前完成停车 |
| DMA SPI3 / 按传输完成 | 仅完成标志/回调 | 2 | DMA buffer → IMU 原始双缓冲 | 低于三级控制中断，不占用 CPU 等待总线 |
| DMA ADC2 / 按扫描完成 | 发布五路平均原始值 | 2 | DMA buffer → 电感快照 | 低于 TIM4，保证下一轮可取得新样本 |
| DMA ADC1 / 低频完成 | 发布电池原始值 | 0 | DMA buffer → 电池快照 | 电池采样不属于紧密闭环 |
| UART TX DMA / 按发送完成 | 推进 TX 环形队列 | 0 | 队列 → UART | 日志/报文输出绝不影响控制时序 |

STC32 的 TIM3/TIM4 优先级位于 `IP3/IP3H`。逐飞中断包装层原先没有暴露这两个入口；现已在 `zf_common_interrupt.[ch]` 增补 `TIM3_IRQn` 与 `TIM4_IRQn`，并由 `service_speed_init()`、`app_inductor_preprocess_init()` 设为 3 级。SPI3/ADC2 DMA 的配置位使用 2 级，确保它们既不会因长期 0 级导致数据陈旧，也不会抢占三级控制 ISR。

## 4. 数据流和控制所有权

```text
编码器 ──TIM3──> speed snapshot ──┐
ADC2 五路 ─DMA──> inductor snapshot ─TIM4──> normalized inductor ─┐
SPI3 全量 IMU ─DMA──> raw double buffer ─TIM7──> imu/element input ─┤
                                                                   ▼
                                                     TIM6: preprocess → element
                                                           → feedforward → plan
                                                                   ▼
                                                           speed targets
                                                                   ▼
                                           TIM5: speed PID → motor PWM / suction PWM

无线/报文/启动/低压 ──后台或TIM8──> request mailbox ────────────────┘
```

关键所有权规则：

- **采集者只发布快照**：DMA ISR 只复制少量已完成数据并置有效标志；不做滤波、打印、IIC 或 PID。
- **控制者只读稳定快照**：TIM3、TIM4、TIM6、TIM7、TIM8 的控制步骤不等待外设完成。
- **执行者只有 TIM5**：异步上下文不得直接改电机/负压 PWM；它们只能写 stop/start/目标值邮箱。
- **诊断在后台**：`printf`、`wprint`、解析、调试输出、蜂鸣器事件打印均不在控制 ISR 中执行。

这把“低优先级队列的执行时刻”从控制闭环中完全移除，满足高实时任务必须由 TIM 中断执行的约束。

## 5. IMU：由部分读取改为全量 DMA 数据帧

### 5.1 帧定义和启动过程

`bsp_imu.c` 使用 SPI3 DMA 传输 15 字节：第一个字节是 `OUT_TEMP_L` 读命令，随后完整接收 14 字节。

| 数据 | 原始字节数 | 是否在运行期读取 |
| --- | ---: | --- |
| 温度 | 2 | 是 |
| Gyro X/Y/Z | 6 | 是 |
| Accel X/Y/Z | 6 | 是 |
| 合计有效数据 | 14 | 是 |

上电配置同样是后台 SPI DMA 状态机：电源稳定等待 → WHO_AM_I → 软复位 → 复位等待 → 寄存器配置 → ready。所有延时由 `service_timetick_what()` 进行状态比较，而非 busy-wait。

### 5.2 正常帧与故障恢复

- DMA 完成回调把完整帧写入双缓冲并仅发布序号。
- TIM7 取到新序号后做整数转浮点、比例换算、滤波与元素 IMU 步骤，再立即提交下一帧。
- 静态零偏累积只在每 200 帧做一次除法；每帧不做不必要的校准除法。
- 正常传输与启动传输都持有 deadline；超过 5 ms 没有 DMA 完成会终止 SPI DMA、释放 CS、记录错误并重试/失败退出，避免“DMA 永远 busy”。

选择 DMA 而不是在 TIM7 直接同步读寄存器的理由是：全量 14 字节在总线上需要确定的传输时间，但这段时间不应成为 1 ms 控制 ISR 的阻塞时间。TIM7 消费上一帧稳定数据，时延固定且可测。

## 6. 通信逐飞库重新审查结果

### 6.1 UART、`printf`、`wprint`

`zf_driver_uart.c` 为 UART1~8 维护独立的 512 B xdata TX 环形缓冲：

- `uart_write_buffer()` 保持兼容签名，但内部转到 `uart_write_buffer_async()`；
- 发送 API 只复制可接受的数据、发布 head 并启动一个连续 DMA 段，**不等待 DMA 完成**；
- DMA TX ISR 推进 tail 并启动下一段；
- 用短 EA 临界区保护 head/tail 的发布，复制较长日志时保持中断开启；
- 有显式 drop counter，队列饱和时丢弃日志而不阻塞控制。

`wprint()` 使用独立格式化缓存和重入保护，再提交无线 UART 环形队列。`printf` 的物理输出同样异步。格式化本身仍有 CPU 成本，因此项目约束为：不得从 TIM ISR 调用 `printf`/`wprint`，控制事件先置位，`app_element_pump_events()` 与 `app_battery_guard_pump_events()` 在后台输出。

此环形 TX 设计是 SPSC（单生产者、DMA ISR 单消费者）。当前工程满足“生产者只在后台”的规则；将来新增 ISR 日志时，必须改为单独的 ISR 安全队列，不能直接并发调用当前 API。

### 6.2 硬件 IIC：P77/P76

硬件 IIC1 CH3 被明确复用到：

```text
P77 = SCL（开漏）
P76 = SDA（开漏 + 数字输入）
```

`iic_async_transfer()` 提交一次 7-bit 事务；`iic_async_process_all_timed()` 在后台每次只推进一个外设事件。ToF 的寄存器读取/写入 buffer 都是静态 xdata，故在异步完成前有效。

超时不是“调用状态机 N 次”，而是比较 TIM0 的 100 µs 真实 tick：持续 100 tick（10 ms）未推进即生成 STOP、清状态并报告错误。这避免了主循环变快时把有效的 40 kHz IIC 事务误判为超时。

DL1B 服务流程为：XSHUT 上下电时序 → boot → firmware/model 读取 → 静态配置表写入 → GPIO ready 轮询 → range status/距离读取。所有阶段均为后台状态机；控制 ISR 从不等待 ToF 总线。

### 6.3 SPI

新增 API：

```c
uint8 spi_dma_async_transfer(...);
uint8 spi_dma_async_is_busy(...);
uint8 spi_dma_async_abort(...);
void  spi_dma_async_irq_handler(...);
```

异步 API 使用 completion IRQ + TX + RX DMA；SPI3 完成 IRQ 为优先级 2，低于 TIM3/4/5/6/7/8 的优先级 3。中断只判定完成/错误、停止 DMA、清 active、调用极短回调。

### 6.4 兼容 API 隔离

逐飞库中仍保留 `spi_dma_transfer()`、同步 IIC、软件 IIC 和 `adc_convert()` 等历史 API，原因是设备库兼容和外部工程 API 稳定性。它们含轮询/等待，**不允许进入当前车辆的运行时控制路径**。

本次调用图复核确认：运行期 IMU 仅调用 `spi_dma_async_transfer()`；ToF 仅调用 `iic_async_transfer()`；电感/电池仅启动 ADC DMA。未来代码评审的硬规则为：任何 TIM ISR 中发现这些同步 API、`while(!flag)`、`system_delay_*` 或日志格式化，即视为阻塞缺陷。

## 7. ADC 与电感计算的性能处理

### 7.1 电感

ADC2 一次硬件 DMA 扫描覆盖 P03~P07 的五个通道，每通道 8 个采样并使用硬件平均值。DMA ISR 只发布五个 `uint16` 快照；TIM4 完成：

1. 写入 7 点历史；
2. 求和并去掉最大/最小；
3. 乘以预计算 `0.2` 替代除以 5；
4. 乘以预计算量程倒数得到 0~100 归一化值；
5. 提交下一次 DMA，不等待完成。

无线标定值的浮点预计算在后台完成；结果通过短临界区一次性发布。TIM4 永远只能看到完整的 `min/max/range_inv` 三元组。若 `max <= min`，倒数强制为 0，输出保持安全的 0，避免无符号下溢后得到错误大范围。

### 7.2 电池

ADC1 同样使用 8 次采样 + 硬件平均 DMA。后台以低频请求并缓存原始值；TIM8 直接比较 raw 阈值 1236（对应 11 V 分压换算），低压即投递 `STOP_ALL`。这样安全停机不等待浮点换算和 ADC 完成。

## 8. 耗时点清单与优化理由

| 原风险 | 后果 | 当前处理 | 剩余边界 |
| --- | --- | --- | --- |
| 同步 SPI / IIC 等待 | 定时器被总线速度、ACK 或故障无限拉长 | DMA / 单步状态机 + deadline | 兼容同步 API 仍存在，禁止新运行时调用 |
| UART 发送等待波特率 | 低速串口可拖慢毫秒级控制 | DMA TX 环形队列 | 后台格式化仍耗 CPU，限制输出频率 |
| TIM4 串行 ADC | 多路/多次采样导致 1 ms 抖动 | ADC2 硬件扫描 DMA | 需要示波器测量最终 ISR WCET |
| 仅读部分 IMU | 姿态/元素算法缺温度或加速度信息 | 每帧 14 B 全量有效数据 | 传感器配置要与量程常量保持一致 |
| per-frame 浮点除法 | 1 ms ISR 中不必要的 DPU/TFPU 开销 | 常量倒数、每 200 帧校准一次 | 动态分母仍只在确有动态比例时保留 |
| 队列执行控制任务 | 时延由后台负载决定 | 控制函数改为 TIM 调用，队列仅保留兼容延时动作 | 新增功能必须遵循 TIM/后台分域 |
| 长 EA 临界区 | 禁止控制定时器造成抖动 | UART 只在元数据发布时关中断；快照仅做短拷贝 | 标定发布偶发，仍需保持短小 |
| DMA IRQ 丢失 | 永久 busy、传感器冻结 | IMU/SPI deadline + abort | IIC 使用真实时间超时，硬件故障要记录计数 |

已消除的确定性等待比微优化更重要；在此基础上才把高频路径的除法变成预计算倒数和乘法。此顺序能保证“快”不以状态完整性或故障恢复为代价。

## 9. DPU32 / TFPU 移植与可用性

供体目录 `C:\Users\20708\Downloads\STC32_DSP32_HUGE` 经逐文件检查后，唯一与本项目 C251 数学运行时相关的源是 `STC32_DSP32.ASM`。它导出了：

```text
?C?ULDIV?    无符号 long 除法
?C?ULIDIV?   无符号 long/int 除法
?C?SLDIV?    有符号 long 除法
?C?SIDIV?    有符号 long/int 除法
```

它们通过 STC `DPUOP` 执行硬件 DPU32 运算。原工程已经有 `sys_tfpu.asm`，并通过 `tfpu_int2float`、`tfpu_mul`、`tfpu_div` 等接口服务浮点路径。

为匹配本工程的 `LARGE ROM(HUGE)` 内存模型，没有直接链接供体预编译库，而是将等价汇编加入：

```text
project/L0_SYS/sys_dpu32.asm
```

并加入 `seekfree.uvproj` 和 `out_file/SEEKFREE.lnp`。最新链接 map 的证据：

```text
out_file/sys_dpu32.obj (STC_DPU32)
?C?SIDIV?   00FDFFF9H
?C?SLDIV?   00FDFFF5H
?C?ULDIV?   00FDFFEBH
?C?ULIDIV?  00FDFFEFH
```

因此 DPU32 对 C251 运行时整数除法已可用；TFPU 对浮点转换/乘除已可用。供体中没有 FFT、FIR、向量 MAC 或通用“DSP 算法库”，故本次没有伪造“已移植完整硬件 DSP”的结论。若后续需要 FIR/FFT，应以目标算法、采样率、定点格式和 DPU 指令集单独立项设计。

## 10. 安全性与并发检查

### 10.1 已落实的安全规则

- 速度、IMU、电感、电池、速度输出、前馈数据均通过 EA 短临界区复制，避免多字节 float 被 ISR/后台撕裂。
- 停车/启动/负压等跨域动作采用 request mailbox；TIM5 是唯一执行点。
- UART 队列和 IIC/SPI 状态机均有可观测的错误或丢弃计数，失败不会转换成无限等待。
- IIC 与 IMU 缓冲均为静态 xdata，生命周期覆盖异步传输。
- 包解析每轮最多处理有限 chunk；兼容函数队列每轮有执行上限；两者不承载控制闭环。
- 电感标定非法量程采用失败安全输出，而不是依赖调用者输入正确。
- `printf`/`wprint` 已从控制 ISR 路径移除，元素与低压日志延后到后台 pump。

### 10.2 必须持续遵守的约束

1. 禁止从任意 TIM ISR 调用同步 SPI/IIC/ADC/UART、`service_delay_*`、`system_delay_*`、`printf` 或 `wprint`。
2. 禁止从多个生产者（尤其 ISR）并发调用当前 UART TX 环形 API。
3. 异步 API 的 TX/RX buffer 必须为静态或由调用者保证在 callback 前有效。
4. 新增 DMA 通道时，优先级不得高于三级控制 timer，除非有独立的时序证明。
5. 没有目标板 WCET 测量前，不得把“0 warning / 0 error”宣传为整车实时性证明。

## 11. 构建与静态验证记录

### 11.1 当前构建结果

在 `project/mdk` 使用 C251 5.60 和 L251 4.66 链接：

```text
C251 COMPILATION COMPLETE.  0 WARNING(S),  0 ERROR(S)
L251 RUN COMPLETE.  0 WARNING(S),  0 ERROR(S)

Program Size:
data=8.6
edata+hdata=8204
xdata=46361
const=18173
code=202632
```

本次最终构建至少重新编译并纳入链接的关键对象包括：

- `zf_common_interrupt.obj`、`zf_driver_uart.obj`、`zf_driver_iic.obj`、`zf_driver_spi.obj`；
- `bsp_imu.obj`、`bsp_inductor.obj`、`bsp_battery.obj`、`bsp_tof_async.obj`；
- `service_speed.obj`、`service_imu.obj`、`service_tof.obj`；
- `app_inductor_preprocess.obj`、`app_feedforward.obj`、`app_speed_plan.obj`、`app_speedout.obj`、`app_motion_postprocess.obj`、`app_battery_guard.obj`；
- `main.obj`、`isr.obj`、`sys_dpu32.obj`。

### 11.2 静态调用复核

静态扫描覆盖了 `project` 与 `libraries/zf_driver` 中的：

- `while` 轮询、`service_delay_*`、`system_delay_*`；
- `uart_write_buffer`、`printf`、`wprint`；
- 同步 `spi_dma_transfer`、IIC read/write、`adc_convert`；
- `app_scheduler_add` 和 `service_function_queue_add`；
- 所有 `pit_ms_init` / `pit_us_init` 注册点。

结论是：历史兼容实现仍会被链接，但不在当前运行时控制调用图上。唯一后台调度任务为启动状态机；它只发布意图。该结论会在两轮独立审查中再次验证。

## 12. 目标板验收清单（尚需执行）

| 验收 | 方法 | 通过判据 |
| --- | --- | --- |
| TIM3/TIM4/TIM5/TIM6/TIM7 WCET | 各 ISR 首尾翻转独立 GPIO，示波器统计最大高脉宽 | 任一 ISR 的最坏宽度远小于其周期，并留出嵌套余量 |
| SPI3 IMU 完整帧 | 逻辑分析仪抓 CS/SCK/MOSI/MISO；串口后台打印帧序号、温度与六轴 | 每帧 15 字节，序号连续，超时计数为 0 |
| IIC DL1B | 观察 P77/P76 和 `tof_diag` | ACK 正常，`fw=3`、`model=234`，距离可更新 |
| UART 压测 | 后台持续日志 + 控制 GPIO 时序 | UART drop 可观测；控制 ISR 宽度不随波特率增大 |
| ADC2 | 观察 DMA 完成和 TIM4 输出 | 五路值更新，无逐通道等待造成的 1 ms 越界 |
| 异常 SPI | 临时屏蔽 SPI3 完成或断开 IMU | 5 ms 内 abort/retry，无永久 busy、无控制中断停滞 |
| 低压停机 | 可控电源慢降 | raw < 1236 后在下一 TIM5 周期停止电机/负压 |
| DPU32 | 加入一次 long 除法自测/调试值 | 结果正确且 map 仍指向 `sys_dpu32.obj` |

## 13. 独立审核—订正循环

用户要求每版三名子智能体独立审核，且审核理由与结论都要被审查。执行规则：每位审查者只读全局源码、diff、构建报告和本 Markdown；报告按“阻断问题 / 非阻断建议 / 证据”给出结论；发现有效阻断项后先修复、重新编译链接，再启动一批**新的**三名审查者。

### V1 实施版

| 审查角色 | 偏好 | 结论 | 处理 |
| --- | --- | --- | --- |
| A：实时与控制链 | 定时、优先级、队列越界 | 待执行 | 待执行 |
| B：通信与并发 | DMA、状态机、缓冲生命周期、临界区 | 待执行 | 待执行 |
| C：C251/DSP 与安全 | ABI、链接、边界值、故障恢复 | 待执行 | 待执行 |

### V2 订正版（仅在 V1 问题修复并重建后启动）

| 审查角色 | 偏好 | 结论 | 处理 |
| --- | --- | --- | --- |
| A2：实时与控制链 | 待执行 | 待执行 | 待执行 |
| B2：通信与并发 | 待执行 | 待执行 | 待执行 |
| C2：C251/DSP 与安全 | 待执行 | 待执行 | 待执行 |

只有 V2 三位审查者都没有剩余阻断项，本报告才会把最终状态标为“全通过”。

## 14. 修改文件索引

核心实现改动位于：

```text
libraries/zf_common/zf_common_interrupt.[ch]
libraries/zf_driver/zf_driver_uart.[ch]
libraries/zf_driver/zf_driver_iic.[ch]
libraries/zf_driver/zf_driver_spi.[ch]
project/L0_SYS/sys_dpu32.asm
project/L1_BSP/bsp_imu.[ch]
project/L1_BSP/bsp_inductor.[ch]
project/L1_BSP/bsp_battery.[ch]
project/L1_BSP/bsp_tof_async.[ch]
project/L2_SERVICE/service_imu.[ch]
project/L2_SERVICE/service_speed.[ch]
project/L2_SERVICE/service_tof.[ch]
project/L3_APP/app_inductor_preprocess.c
project/L3_APP/app_motion_preprocess.[ch]
project/L3_APP/app_motion_postprocess.[ch]
project/L3_APP/app_feedforward.[ch]
project/L3_APP/app_speed_plan.[ch]
project/L3_APP/app_speedout.[ch]
project/L3_APP/app_element.[ch]
project/L3_APP/app_battery_guard.[ch]
project/user/main.c
project/user/isr.c
project/mdk/seekfree.uvproj
project/mdk/out_file/SEEKFREE.lnp
```

本文件会随 V1/V2 审核结果继续更新，作为后续维护的实时性和并发契约。
