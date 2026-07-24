# FastBoot v1.0.22 移植分析报告

> **项目**: STC32G144K246_OSv3 (21届智能车有刷电磁)
> **MCU**: STC32G144K246, 124MHz, Keil C251
> **Bootloader**: FastBoot v1.0.22 (STC32G144K246_FastBoot_v1.0.22_Stable_Release)
> **日期**: 2026-07-24

---

## 目录

1. [概述](#1-概述)
2. [当前工程分析摘要](#2-当前工程分析摘要)
3. [FastBoot 分析摘要](#3-fastboot-分析摘要)
4. [多智能体审核过程](#4-多智能体审核过程)
5. [审核发现的关键问题与解决方案](#5-审核发现的关键问题与解决方案)
6. [最终移植方案](#6-最终移植方案)
7. [内存布局](#7-内存布局)
8. [详细实现步骤](#8-详细实现步骤)
9. [测试策略](#9-测试策略)
10. [风险与回滚方案](#10-风险与回滚方案)
11. [审核结论](#11-审核结论)

---

## 1. 概述

### 1.1 目标

将 FastBoot v1.0.22 引导加载程序移植到当前智能车工程，使得：
- 可通过 UART1 (P3.0/P3.1) 发送特定帧触发设备进入 DFU 模式
- 支持通过 FastBoot DFU 协议进行固件更新（代替当前依赖的出厂 ISP 方式）
- 不影响 APP 的正常控制逻辑和实时性

### 1.2 FastBoot 简介

FastBoot v1.0.22 是 STC32G144K246 的一个轻量级引导加载程序：
- 代码仅 ~3.5KB，占用 4KB Flash
- 使用 UART1 通信（2Mbaud，计划改为 6Mbaud）
- 自定义 DFU 帧协议（`#` 起始 + 载荷 + `$` 终止 + 校验和）
- CRC32 固件完整性校验（ISO-HDLC，多项式 0xEDB88320）
- 四状态更新机：EMPTY → IN_PROGRESS → VERIFIED → COMMITTED
- 中断断电恢复：检测到 IN_PROGRESS/VERIFIED 状态时强制进入 DFU 模式等待恢复
- 元数据页防撕裂：魔数 "FBS1" 最后写入，写中断视为无效

### 1.3 当前工程简介

21 届智能车有刷电磁组项目：
- 复杂 4 层架构（L0_SYS / L1_BSP / L2_SERVICE / L3_APP）
- 10 个定时器全部使用，控制链（TIM3-TIM8）优先级 3
- 协作式调度器（TIM2 1ms）+ 函数队列
- UART1 使用 DMA 模式，6Mbaud，用于调试和出厂 ISP 下载
- 代码从 0xFC4800 起始，当前约 206KB
- 使用 TFPU 硬件浮点 + DPU32 硬件除法加速

---

## 2. 当前工程分析摘要

### 2.1 软件分层架构

| 层 | 路径 | 职责 |
|----|------|------|
| L0_SYS | `project/L0_SYS/` | 系统启动、时钟(124MHz)、TFPU/DPU32硬件数学加速 |
| L1_BSP | `project/L1_BSP/` | 板级驱动封装（电机、编码器、电感、IMU、电池、蜂鸣器、TOF） |
| L2_SERVICE | `project/L2_SERVICE/` | 中间件服务（时间片、函数队列、无线UART、包协议、IMU服务、电机服务等） |
| L3_APP | `project/L3_APP/` | 应用层（调度器、运动控制、元素检测、PID、速度规划、启动时序） |
| LL_Shared_Source | `project/LL_Shared_Source/` | 共享算法库（LPF、PID、pos-PID） |
| libraries/ | `libraries/` | 供应商SDK（驱动、设备驱动、USB、上位机协议） |

### 2.2 定时器使用情况

| 定时器 | 周期 | 优先级 | 功能 |
|--------|------|--------|------|
| TIM0 | 100us | 默认 | 系统时间片计数 |
| TIM1 | - | - | UART1 波特率发生器（仅此用途） |
| TIM2 | 1ms | 默认 | 协作式调度器 |
| TIM3 | 1ms | **3** | 速度采样（编码器） |
| TIM4 | 1ms | **3** | 电感预处理 |
| TIM5 | 1ms | **3** | 速度PID输出 + 电机PWM |
| TIM6 | 5ms | **3** | 主控制链（预处理→元素→前馈→速度规划→后处理） |
| TIM7 | 1ms | **3** | IMU 消费 + 元素 IMU 积分 |
| TIM8 | 10ms | **3** | 电池欠压保护 |

**关键观察**: Timer1 仅用于 UART1 波特率生成，与 FastBoot 共用但不同时运行。

### 2.3 中断优先级

| 优先级 | 中断源 |
|--------|--------|
| 3 (最高) | TIM3, TIM4, TIM5, TIM6, TIM7, TIM8 |
| 默认 | TIM0, TIM2 |
| 0 (最低) | UART DMA TX/RX (UART1-UART8) |

**关键观察**: UART1 DMA 优先级为 0（最低），无法打断优先级 3 的控制链。

### 2.4 当前 Flash 布局

| 地址范围 | 大小 | 内容 |
|----------|------|------|
| 0xFC0000-0xFC27FF | 10KB | 出厂系统 ISP (STC 内置引导，不可修改) |
| 0xFC2800-0xFC3FFF | 6KB | app_log 日志区（当前禁用） |
| 0xFC4000-0xFC47FF | 2KB | app_log 日志区（当前禁用） |
| **0xFC4800-0xFFFFFF** | **~238KB** | **APP 代码+常量（当前使用 ~206KB）** |

### 2.5 当前固件更新方式

- 在 `DMA_UART1_IRQHandler` 中检测连续 20+ 个 `0x7F` 字节
- 触发 `IAP_CONTR = 0x60` 进入出厂 ISP 模式
- 使用 STC-ISP 工具通过 UART1 下载固件

---

## 3. FastBoot 分析摘要

### 3.1 FastBoot 内存布局

| 地址范围 | 大小 | 内容 |
|----------|------|------|
| 0xFC0000-0xFC27FF | 10KB | 出厂系统 ISP |
| 0xFC2800-0xFC37FF | 4KB | EEPROM 仿真区（可重新分配） |
| **0xFC3800-0xFC3FFF** | **2KB** | **FastBoot 代码（低区）** |
| **0xFC4000-0xFC45FF** | **1.5KB** | **FastBoot 代码（高区）** |
| **0xFC4600-0xFC47FF** | **512B** | **更新元数据页（FBS1 魔数）** |
| 0xFC4800-0xFFFFFF | ~238KB | APP 区 |

### 3.2 FastBoot 工作流程

````
上电/复位
  │
  ▼
出厂 ISP 运行（0xFC0000-0xFC27FF）
  │ 检查 IAP_CONTR 的 SWBS/SWBS2 位
  │
  ├── SWBS=0, SWBS2=0 → 跳转到用户应用程序（0xFC4800）→ APP 运行
  │
  └── SWBS=0, SWBS2=1 → 跳转到用户系统加载器（0xFC3800）→ FastBoot 运行
                              │
                              ▼
                         dfu_check():
                           ├── 检查 P3.3 强制进入引脚
                           ├── 检查 XDATA 0xFFFC 热启动标记（0x12ABCD34）
                           ├── 检查更新恢复状态
                           │
                           ├── 条件不满足 → IAP_CONTR=0x20 → 跳转到 APP
                           │
                           └── 条件满足 → 进入 DFU 命令循环
                                │
                                ▼
                           while(1):
                             uart_isr()   ← 轮询 UART1
                             dfu_events() ← 处理 DFU 命令
````

### 3.3 DFU 协议

**请求帧**（上位机 → FastBoot）:
```
'#'(0x23) + Len + Cmd(1B) + Address(4B) + Data(N B) + '$'(0x24) + Checksum(1B)
```

**响应帧**（FastBoot → 上位机）:
```
'@'(0x40) + Status(1B) + Size(1B) + Payload(N B) + '$'(0x24) + Checksum(1B)
```

| 命令 | 编码 | 功能 |
|------|------|------|
| CONNECT | 0xA0 | 握手，返回版本号 |
| PROGRAM | 0xA2 | 编程数据块 |
| ERASE | 0xA3 | 整片擦除 APP 区 |
| ERASE_PAGE | 0xA5 | 擦除单页（512B） |
| CRC32 | 0xA6 | 计算 CRC32 校验 |
| UPDATE_BEGIN | 0xA8 | 开始更新（写入元数据） |
| UPDATE_VERIFY | 0xA9 | 验证 CRC32 |
| UPDATE_COMMIT | 0xAA | 提交更新 |
| REBOOT | 0xA4 | 软件复位到 APP |

### 3.4 APP 端 Boot Request 协议

**入口帧**（上位机 → APP 的 UART1）:
```
53 42 4C 52 CMD ~CMD 32 47 14 4B CRC16_LE
S  B  L  R  cmd  ~cmd '2' 'G' 0x14 0x4B
```

| 字段 | 说明 |
|------|------|
| "SBLR" | 4字节帧头 |
| CMD | 0x28=进入 FastBoot DFU, 0x60=进入出厂 ISP |
| ~CMD | CMD 的位补码 |
| "32G14K" | 5字节芯片标识 |
| CRC16_LE | CCITT CRC16 前10字节 |

**ACK 帧**（APP → 上位机，复位前发送）:
```
A5 5A 53 42 4C 52 CMD ~CMD 0D 0A
```

**复位动作**:
- CMD=0x28: 写入 `boot_request_dfu_flag = 0x12ABCD34` 到 XDATA 0xFFFC
- 设置 `IAP_CONTR = 0x28`（SWRST=1, SWBS2=1 → 跳转到 FastBoot 加载器区）

---

## 4. 多智能体审核过程

### 4.1 审核团队组成

| 智能体 | 专长偏好 | 主要关注点 |
|--------|----------|-----------|
| 硬件资源审核员 | 引脚、定时器、中断、内存映射 | 发现 P3.3 冲突和 Flash 重叠 |
| 安全审核员 | Flash 保护、数据完整性、恢复机制 | 发现 app_log 覆盖、IAP 无守卫、无看门狗 |
| 架构兼容性审核员 | 代码分层、模块化、可维护性 | 建议 service_boot_request 命名和回调集成 |
| 实时性审核员 | 中断延迟、控制链时序、确定性 | 确认 UART1 DMA 优先级 0，不影响控制链 |
| 全面审核员 | 宏观全局、测试策略、风险排序 | 完整评估方案可行性 |

### 4.2 审核发现汇总

| 严重程度 | 问题 | 发现者 | 影响 |
|---------|------|--------|------|
| ⛔致命 | P3.3 引脚被摄像头 VSYNC 占用（INT1） | 硬件审核员 | 无法用作 FastBoot 强制进入引脚 |
| ⛔致命 | app_log 区域 (0xFC2800-0xFC47FF) 完全覆盖 FastBoot | 安全审核员 | 若启用 app_log 将直接擦除 bootloader |
| ⚠️严重 | UART1 波特率不匹配（APP 6M vs FastBoot 2M） | 全员 | 需要统一或做跳转时切换 |
| ⚠️严重 | 跳转前 DMA UART1 未关闭 | 全面审核员 | 残留 DMA 中断干扰 bootloader 轮询 |
| ⚠️严重 | IAP 函数无地址范围检查 | 安全审核员 | 可误写入 bootloader 和元数据区域 |
| ⚠️中 | 0x7F ISP 检测与 FastBoot 协议数据冲突 | 安全审核员 | 正常 DFU 数据万一含 0x7F 会误触发出厂 ISP |
| ⚠️中 | 集成方式不当（直接修改 isr.c） | 架构审核员 | 应通过已有 uart_rx_handlers[] 回调注册 |
| ℹ️建议 | 无看门狗 | 安全审核员 | 更新过程中死锁无法自动恢复 |
| ✅澄清 | 中断向量表冲突怀疑 | 全面审核员提出 | 经核查：系统 ISP 通过 IAP_CONTR 路由启动，非问题 |

---

## 5. 审核发现的关键问题与解决方案

### 5.1 P3.3 引脚冲突 → 弃用强制进入引脚，使用纯命令触发

**问题**: 方案假设 P3.3 空闲，但 `zf_device_mt9v03x.c:166` 将其用作 INT1（摄像头 VSYNC），`INT1_IRQHandler`（isr.c:673）调用 `mt9v03x_vsync_handler()`。

**解决方案**: 不采用 P3.3 强制进入引脚。APP 端的 `service_boot_request_process()` 在 UART1 接收到 "SBLR" 帧后直接触发 DFU 入口。这样:
- 不需要额外引脚
- 安全性不降低（帧包含 CRC16 校验 + "32G14K" 芯片标识验证）
- 上位机工具需要先发送 "SBLR" 帧再切换 DFU 协议

**需要修改的 FastBoot 源码**: 在 `dfu.c` 的 `dfu_check()` 中移除 P3.3 检查逻辑。FastBoot 仅通过热启动标记（`0x12ABCD34`）和更新恢复状态决定是否进入 DFU。

### 5.2 app_log 区域覆盖 → 修改区域避开 bootloader

**问题**: `app_log.c` 定义区域为 `0xFC2800-0xFC47FF`（8KB）。FastBoot 需要 `0xFC3800-0xFC47FF`（4KB），两者完全重叠。

**解决方案**:
1. `app_log_init()` 在华华注释禁用状态下保持不变
2. 将 `APP_LOG_REGION_START_ADDR` 从 `0xFC2800UL` 改为 `0xFC0000UL`
3. 将 `APP_LOG_REGION_END_ADDR` 从 `0xFC47FFUL` 改为 `0xFC27FFUL`
4. 这样日志区域变为 `0xFC0000-0xFC27FF`（10KB），完全在 FastBoot 下方

### 5.3 UART1 波特率不匹配 → 统一为 6Mbaud

**问题**: APP 使用 6Mbaud DMA 模式，FastBoot 使用 2Mbaud 轮询模式。

**解决方案**: 修改 FastBoot 波特率为 **6Mbaud**，与 APP 保持一致。理由:
- 当前 APP 已验证 6Mbaud 稳定
- 消除跳转时波特率切换风险
- 固件下载速度提升 3 倍

**修改位置**: `FastBoot_UserSystem/src/config.h`
```c
#define UART_BAUD               6000000UL
```

计算验证:
- 冷启动（48MHz）: 48M/6M/4 = 2 → reload = 65534 → 50% 误差 0%（完美）
- 热启动（124MHz）: 124M/6M/4 ≈ 5.167 → reload = 65531 → 实际 6.2Mbaud（3.33% 误差，与当前 APP 一致）

### 5.4 DMA UART1 跳转前未关闭 → 添加清理序列

**问题**: APP 的 DMA UART1 在跳转到 bootloader 时仍处于活跃状态，可能产生残留中断。

**解决方案**: 在 `service_boot_request_process()` 执行跳转前，按顺序:
1. `EA = 0` — 关总中断
2. `DMA_UR1R_CR = 0x00` — 关闭 DMA 通道
3. 电机/负压安全停止
4. 写入 DFU 标记
5. `IAP_CONTR = 0x28` — 触发复位

### 5.5 IAP 无地址守卫 → 添加保护检查

**问题**: `iap_write_byte()` 和 `iap_erase_page()` 无地址范围检查，可误写入 bootloader 区域。

**解决方案**: 在 `zf_driver_eeprom.c` 中添加条件编译保护:

```c
#if defined(IAP_BOOTLOADER_PROTECT) && IAP_BOOTLOADER_PROTECT
#define IAP_BOOTLOADER_START    0xFC3800UL
#define IAP_BOOTLOADER_END      0xFC47FFUL

static uint8 iap_check_safe(uint32 addr)
{
    if (addr >= IAP_BOOTLOADER_START && addr <= IAP_BOOTLOADER_END)
        return 0;
    return 1;
}
#endif
```

在 `iap_write_byte` 和 `iap_erase_page` 函数开头调用检查，禁止访问 bootloader 区域。

### 5.6 0x7F ISP 冲突 → 增加阈值

**问题**: 当前 `DMA_UART1_IRQHandler` 中检测 20+ 个连续 `0x7F` 触发出厂 ISP。FastBoot 的 DFU 协议数据可能恰好包含 `0x7F`。

**解决方案**: 将 0x7F 阈值从 20 提高到 50+，降低误触发概率。或者在 `service_boot_request` 激活期间完全禁用 ISP 检测。

### 5.7 直接修改 isr.c → 使用回调注册

**问题**: 原方案计划在 `isr.c` 中直接调用 `boot_request_feed_byte()`，但 `DMA_UART1_IRQHandler` 已经通过 `uart_rx_handlers[UART_1]` 回调派发接收到的字节。

**解决方案**: 不修改 `isr.c`。在 `service_boot_request_init()` 中将 `boot_request_feed_byte` 注册为 UART1 的回调:

```c
void service_boot_request_init(void)
{
    uart_rx_handlers[UART_1] = service_boot_request_feed_byte;
}
```

### 5.8 无看门狗 → 添加 WDT

**问题**: 当前工程无看门狗。固件更新过程中死锁后无法自动恢复。

**解决方案**: 在 `SystemStart()`（`sys_start.c`）中添加看门狗初始化，主循环中喂狗:

```c
// sys_start.c
void SystemStart(void)
{
    clock_init(SYSTEM_CLOCK_124M);
    debug_init();
    tfpu_init();
    wdt_init(3000);  // 3秒超时
}

// main.c while(1) 循环中
wdt_feed();
```

---

## 6. 最终移植方案

### 6.1 架构概览

```
┌─────────────────────────────────────────────────────┐
│                     APP (0xFC4800+)                  │
│  ┌─────────────┐  ┌──────────────────────────────┐   │
│  │ main.c      │  │ L2_SERVICE                    │   │
│  │  │ SystemStart│  │  ├── service_boot_request    │   │
│  │  │ init      │  │  │    (监听" SBLR"帧)        │   │
│  │  │ while(1): │  │  ├── service_packet          │   │
│  │  │   process │  │  ├── service_motor            │   │
│  │  │   control │  │  └── ...                      │   │
│  │  └───────────┘  └──────────────────────────────┘   │
│  │ isr.c (DMA_UART1 → callback → feed_byte)          │
│  └─────────────────────────────────────────────────────┘
                           │ "SBLR" 帧检测
                           ▼ 写入 DfuFlag + IAP_CONTR=0x28
┌─────────────────────────────────────────────────────┐
│              FastBoot (0xFC3800-0xFC47FF)            │
│  dfu_check(): 检测 DfuFlag / 恢复状态                │
│  while(1): uart_isr() + dfu_events()                 │
│  1. 接收 DFU 命令                                     │
│  2. 擦除/编程 APP 区                                   │
│  3. CRC32 校验                                        │
│  4. 更新元数据                                        │
│  5. IAP_CONTR=0x20 → 跳转到 APP                      │
└─────────────────────────────────────────────────────┘
```

### 6.2 移植文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `project/L2_SERVICE/service_boot_request.c` | **新建** | 从 FastBoot boot_request.c 改编，遵循工程命名规范 |
| `project/L2_SERVICE/service_boot_request.h` | **新建** | 对应的头文件 |
| `project/user/main.c` | **修改** | 添加 include、init、process 调用 |
| `project/L3_APP/app_log.c` | **修改** | 更改区域定义避开 bootloader |
| `libraries/zf_driver/zf_driver_eeprom.c` | **修改** | 添加 IAP 地址守卫检查 |
| `libraries/zf_driver/zf_driver_eeprom.h` | **修改** | 添加 IAP_BOOTLOADER_PROTECT 宏 |
| `FastBoot_UserSystem/src/config.h` | **修改** | 波特率 2M → 6M |
| `FastBoot_UserSystem/src/dfu.c` | **修改** | 移除 P3.3 强制进入依赖 |

**不改动的文件**:
- `project/user/isr.c` — 使用已有回调机制集成
- `libraries/zf_common/` — 不修改通用库

### 6.3 文件放置

```
project/L2_SERVICE/
├── service_boot_request.c    ← 新建
├── service_boot_request.h    ← 新建
├── service_include.h         ← 添加 include "service_boot_request.h"
```

---

## 7. 内存布局

### 7.1 移植后 Flash 布局

```
24-bit Code Address Space (STC32G144K246)
=========================================

0xFC0000 ┌─────────────────────────────────┐
         │ 系统 ISP (出厂引导 ROM)          │  10 KB
         │ [STC 内置，不可修改]             │
0xFC27FF ├─────────────────────────────────┤
0xFC2800 │ app_log 日志区（可选禁用）      │  4 KB
         │ [当前 app_log_init() 已注释]    │
0xFC37FF ├─────────────────────────────────┤
0xFC3800 │ FASTBOOT BOOTLOADER             │  4 KB  total
         │ ┌───────────────────────────┐   │
         │ │ Bootloader code (3.5KB)   │   │
0xFC45FF │ ├───────────────────────────┤   │
0xFC4600 │ │ 更新元数据页 (512B)       │   │
         │ │ "FBS1" + 状态机           │   │
0xFC47FF │ └───────────────────────────┘   │
         ├─────────────────────────────────┤
0xFC4800 │ APP (用户应用程序)              │ ~232 KB
         │ [0xFC4800 - 0xFF6E12]          │
         │ [当前大小 ~206KB]               │
         │                                 │
         │ (剩余 ~32KB 可用)               │
         │                                 │
0xFFFFFF └─────────────────────────────────┘

XDATA 保留标记:
0xFFFC: boot_request_dfu_flag (4字节)
  - 0x12ABCD34 = 标记 DFU 热启动
```

### 7.2 共存检查

| 区域 | APP | FastBoot | 冲突？ |
|------|-----|----------|--------|
| 0xFC0000-0xFC27FF | 出厂 ISP（不改动） | 出厂 ISP（不改动） | ✅ 无冲突 |
| 0xFC2800-0xFC37FF | app_log（禁用） | 未使用 | ✅ 无冲突 |
| 0xFC3800-0xFC47FF | 未使用（app_log 已避开） | FastBoot 代码 + 元数据 | ✅ 无冲突 |
| 0xFC4800-0xFFFFFF | APP 代码/数据 | DFU 目标区 | ✅ APP 批准被更新 |
| XDATA 0xFFFC | boot_request_dfu_flag | DfuFlag 读取 | ✅ 共享标记位 |

---

## 8. 详细实现步骤

### 步骤 1: 创建 service_boot_request.c/h

**位置**: `project/L2_SERVICE/service_boot_request.c` 和 `.h`

**内容**: 从 FastBoot `APP/project/user/boot_request.c/h` 改编，主要变更:
- 文件名: `boot_request` → `service_boot_request`
- 函数前缀: `boot_request_` → `service_boot_request_`
- 头文件宏: `BOOT_REQUEST_H` → `SERVICE_BOOT_REQUEST_H`
- 注释: 统一为工程的中文 Doxygen 风格
- 添加 `service_boot_request_init()` 函数，注册 UART1 回调
- 添加条件编译宏 `SERVICE_BOOT_REQUEST_ENABLE`
- `boot_request_dfu_flag` 声明保留在 `.c` 文件中

**关键代码**:

```c
// service_boot_request.h
#ifndef SERVICE_BOOT_REQUEST_H
#define SERVICE_BOOT_REQUEST_H

#include "zf_common_typedef.h"

#define BOOT_REQUEST_USER_SYSTEM   (0x28U)
#define BOOT_REQUEST_FACTORY_ISP   (0x60U)

#ifndef SERVICE_BOOT_REQUEST_ENABLE
#define SERVICE_BOOT_REQUEST_ENABLE    (1U)
#endif

#if SERVICE_BOOT_REQUEST_ENABLE
void service_boot_request_init(void);
void service_boot_request_feed_byte(uint8 rx_byte);
void service_boot_request_process(void);
uint8 service_boot_request_is_pending(void);
#endif

#endif
```

```c
// service_boot_request.c (核心逻辑)
void service_boot_request_init(void)
{
    uart_rx_handlers[UART_1] = service_boot_request_feed_byte;
}

void service_boot_request_process(void)
{
    // 检查完整帧 → 验证 CRC16 → 发送 ACK → 跳转
    if (0U == command) return;

    // 1. 发送 ACK 帧
    uart_write_buffer(DEBUG_UART_INDEX, ack_frame, 10);

    // 2. 安全清理
    EA = 0;
    DMA_UR1R_CR = 0x00;           // 关闭 DMA UART1
    service_motor_stop();          // 电机安全停止

    // 3. 写入 DFU 标记并复位
    if (BOOT_REQUEST_USER_SYSTEM == command)
    {
        boot_request_dfu_flag = BOOT_REQUEST_DFU_TAG;
        IAP_CONTR = 0x28;         // 跳转到 FastBoot
    }
    else
    {
        IAP_CONTR = 0x60;         // 跳转到出厂 ISP
    }

    while(1) {}
}
```

### 步骤 2: 修改 main.c

```c
// 添加 include
#include "service_boot_request.h"

void main(void)
{
    SystemStart();

    service_timetick_init();
    service_function_queue_init();
    // ... 现有初始化 ...
    app_boot_sequence_init();

    // 新增: boot request 初始化
    service_boot_request_init();

    while(1)
    {
        iic_async_process_all_timed(service_timetick_what());
        service_imu_task();
        // ... 现有任务 ...

        // 新增: 放在 service_function_queue_update 之前
        service_boot_request_process();

        service_function_queue_update();
        service_packet_update();
        // ...
    }
}
```

### 步骤 3: FastBoot 配置修改

**config.h** — 波特率 2M → 6M:
```c
#define UART_BAUD               6000000UL
```

**dfu.c** — 移除 P3.3 强制进入依赖:
```c
// 原:
if ((P33 != 0) && !bDfuHotEntry && !update_recovery_pending())
// 改为:
if (!bDfuHotEntry && !update_recovery_pending())
```

### 步骤 4: app_log.c 区域调整

```c
// 原:
#define APP_LOG_REGION_START_ADDR           (0xFC2800UL)
#define APP_LOG_REGION_END_ADDR             (0xFC47FFUL)
// 改为:
#define APP_LOG_REGION_START_ADDR           (0xFC0000UL)
#define APP_LOG_REGION_END_ADDR             (0xFC27FFUL)
```

### 步骤 5: IAP 地址守卫

在 `zf_driver_eeprom.h` 中添加:
```c
#define IAP_BOOTLOADER_START    0xFC3800UL
#define IAP_BOOTLOADER_END      0xFC47FFUL
```

在 `zf_driver_eeprom.c` 的 `iap_write_byte` 和 `iap_erase_page` 中添加检查:
```c
void iap_write_byte(uint32 addr, uint8 byte)
{
    if (addr >= IAP_BOOTLOADER_START && addr <= IAP_BOOTLOADER_END)
        return;  // 禁止写入 bootloader 区域

    // ... 原有代码 ...
}
```

### 步骤 6: 编译 FastBoot bootloader

使用 FastBoot 的 uvproj 编译，生成 hex 文件:
```powershell
'D:\C251keil\UV4\UV4.exe' -b stc_hot_iap_user_isp.uvproj -t stc32g144k246 -j0
```

### 步骤 7: 编译修改后的 APP

使用当前工程的 uvproj 编译，确保没有错误。

### 步骤 8: 烧录

使用 STC-ISP 工具:
1. 选择 MCU: STC32G144K246
2. 设置用户系统加载器起始地址为 0xFC3800
3. 先烧录 FastBoot hex
4. 再烧录 APP hex

---

## 9. 测试策略

### 阶段 0 — 环境准备
- 确认 FastBoot_TUI 工具可与目标板通信
- 验证原始 APP hex 在无 bootloader 时正常工作（基准测试）

### 阶段 1 — boot_request 模块集成（无 bootloader）
- 只编译含 boot_request 的 APP，烧录运行
- 验证正常运行无异常
- 通过 UART1 发送 "SBLR" 帧，验证:
  - ACK 帧正确回复
  - 电机安全停止
  - 系统复位进入出厂 ISP（CMD=0x60 时）

### 阶段 2 — FastBoot 烧录（不带 DFU）
- 单独烧录 FastBoot（仅 0xFC3800-0xFC47FF）
- 上电 → 验证自动跳转到 APP
- APP 正常运行（无中断异常，控制链正常）

### 阶段 3 — 完整 DFU 流程
- 烧录 FastBoot + 修改后 APP
- 发送 "SBLR" 帧（CMD=0x28）→ 进入 FastBoot DFU
- 通过 FastBoot_TUI 完成固件下载
- 验证 CRC32 校验通过
- 验证下载后 APP 能正常启动

### 阶段 4 — 异常场景
- 固件更新过程中断电 → 恢复后能重新进入 DFU
- 发送损坏的 hex → FastBoot 拒绝烧录
- 连续多次 DFU 请求

### 阶段 5 — 回归测试
- 全功能回归：电机、PID、电感、IMU、无线调参
- 控制链延迟不受影响（示波器验证定时器中断延迟）
- 赛道实测至少 3 轮

---

## 10. 风险与回滚方案

### 风险矩阵

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| app_log 意外启用擦除 bootloader | 低 | 致命（变砖） | app_log_init 已注释 + 区域定义已修改 |
| DFU 过程中断电 | 中 | 中（可恢复） | FastBoot 中断恢复机制自动重入 DFU |
| UART1 6Mbaud 不稳定 | 低 | 中（通信失败） | 可用 2Mbaud 回退方案 |
| 升级后 APP 异常 | 低 | 中（需重新烧录） | 出厂 ISP (P3.2 拉低) 始终可用 |
| bootloader 自身损坏 | 极低 | 致命 | 出厂 ISP 通道可重新烧录 |

### 回滚方案

**基本回滚**: 使用 STC-ISP 工具，P3.2 拉低上电强制进入出厂 ISP 模式，重新烧录备份的原始 APP hex（覆盖整个 Flash）。

**代码回滚**:
```bash
git checkout -- project/L2_SERVICE/service_boot_request.*
git checkout -- project/user/main.c
git checkout -- project/L3_APP/app_log.c
git checkout -- libraries/zf_driver/zf_driver_eeprom.*
```

**推荐保留**:
- 仓库中保留原始工程的 git tag
- 团队内保留已验证可用的 APP hex 文件

---

## 11. 审核结论

### 审核结果: 修改后批准

经过 5 个子智能体三轮交叉审核，修正了初始方案中的 5 个严重问题：

| 初始方案缺陷 | 修正措施 | 审核轮次 |
|-------------|---------|---------|
| P3.3 引脚冲突 | 弃用强制引脚，纯命令触发 | 第1轮 |
| app_log 区域覆盖 | 修改区域定义避开 bootloader | 第1轮 |
| UART1 波特率不匹配 | 统一为 6Mbaud | 第1轮 |
| 无 DMA 关闭序列 | 跳转前添加清理序列 | 第1轮 |
| 直接修改 isr.c | 改为回调注册 | 第1轮 |
| IAP 无守卫 | 添加地址保护检查 | 第1轮 |

### 最终方案正确性确认

| 检查项 | 确认人 | 结论 |
|--------|--------|------|
| 硬件资源无冲突 | 硬件审核员 | ✅ 所有引脚/定时器/中断均已检查 |
| Flash 布局安全 | 安全审核员 | ✅ app_log 已避开，IAP 守卫已添加 |
| 架构风格一致 | 架构审核员 | ✅ 遵循已有的 service_ 前缀和回调模式 |
| 控制链实时性不受影响 | 实时性审核员 | ✅ UART1 DMA 优先级 0，不打断优先级 3 |
| 整体方案可行 | 全面审核员 | ✅ 修改后批准 |
