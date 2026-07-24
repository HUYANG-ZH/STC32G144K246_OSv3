/*********************************************************************************************************************
* STC32G144K Opensource Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是 STC32G144K 开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU 通用公共许可证）的条款
* 即 GPL 的第 3 版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅 <https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议，以上许可声明为译文版本
* 许可声明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下，即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序，但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_driver_iic.h
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件的版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K246
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2026-07-19        OpenAI         first version，支持 I2C1/I2C2、超时与总线恢复
********************************************************************************************************************/
#ifndef _zf_driver_iic_h_
#define _zf_driver_iic_h_

#include "zf_driver_gpio.h"

typedef enum
{
    IIC_1 = 0,
    IIC_2,
    IIC_NUM,
} iic_index_enum;

// 该枚举禁止用户修改。同一组 SCL/SDA 必须成对使用，禁止跨组混用。
typedef enum
{
    // ---------------- IIC_1 ---------------- //
    IIC1_CH1_SCL_P15 = 0x0000 | IO_P15,
    IIC1_CH1_SDA_P14 = 0x0000 | IO_P14,

    IIC1_CH2_SCL_P25 = 0x0100 | IO_P25,
    IIC1_CH2_SDA_P24 = 0x0100 | IO_P24,

    IIC1_CH3_SCL_P77 = 0x0200 | IO_P77,
    IIC1_CH3_SDA_P76 = 0x0200 | IO_P76,

    IIC1_CH4_SCL_P32 = 0x0300 | IO_P32,
    IIC1_CH4_SDA_P33 = 0x0300 | IO_P33,
    // ---------------- IIC_1 ---------------- //

    // ---------------- IIC_2 ---------------- //
    IIC2_CH1_SCL_P27 = 0x1000 | IO_P27,
    IIC2_CH1_SDA_P26 = 0x1000 | IO_P26,

    IIC2_CH2_SCL_P37 = 0x1100 | IO_P37,
    IIC2_CH2_SDA_P36 = 0x1100 | IO_P36,

    IIC2_CH3_SCL_P17 = 0x1200 | IO_P17,
    IIC2_CH3_SDA_P16 = 0x1200 | IO_P16,

    IIC2_CH4_SCL_P61 = 0x1300 | IO_P61,
    IIC2_CH4_SDA_P60 = 0x1300 | IO_P60,
    // ---------------- IIC_2 ---------------- //

    IIC_NULL_PIN = 0xFFFF,
} iic_pin_enum;

typedef enum
{
    IIC_SUCCESS = 0,
    IIC_ERROR_INVALID_PARAMETER,
    IIC_ERROR_NOT_INITIALIZED,
    IIC_ERROR_BUSY,
    IIC_ERROR_BUS_STUCK,
    IIC_ERROR_NACK,
    IIC_ERROR_TIMEOUT,
} iic_status_enum;

// 单个硬件命令的软件超时循环次数。可按项目主频与允许的 clock stretching 时间覆盖该宏。
#ifndef IIC_TIMEOUT_COUNT
#define IIC_TIMEOUT_COUNT               (1000000UL)
#endif

// 总线恢复时 SCL 脉冲宽度，单位 us。
#ifndef IIC_RECOVERY_PULSE_US
#define IIC_RECOVERY_PULSE_US           (5U)
#endif

/* 异步状态机等待单个硬件命令完成的最大 service 次数；每次 service 为 O(1)，绝不轮询等待。 */
#ifndef IIC_ASYNC_TIMEOUT_TICKS
#define IIC_ASYNC_TIMEOUT_TICKS          (100UL)
#endif

/* Compatibility-only no-timebase servicing; runtime must use the timed API. */
#ifndef IIC_ASYNC_MAX_SERVICE_COUNT
#define IIC_ASYNC_MAX_SERVICE_COUNT      (5000UL)
#endif

// ================================ 带状态返回的底层接口 ================================ //
iic_status_enum iic_init                 (iic_index_enum iic_n, uint8 addr, uint32 speed, iic_pin_enum scl_pin, iic_pin_enum sda_pin);
iic_status_enum iic_write                (iic_index_enum iic_n, uint8 addr, const uint8 *buffer, uint32 len);
iic_status_enum iic_read                 (iic_index_enum iic_n, uint8 addr, uint8 *buffer, uint32 len);
iic_status_enum iic_transfer             (iic_index_enum iic_n, uint8 addr, const uint8 *write_data, uint32 write_len, uint8 *read_data, uint32 read_len);
iic_status_enum iic_recover_bus          (iic_index_enum iic_n);
iic_status_enum iic_get_last_status      (iic_index_enum iic_n);
uint32          iic_get_actual_speed     (iic_index_enum iic_n);
void            iic_deinit               (iic_index_enum iic_n);

// ================================ 非阻塞硬件 IIC 接口 ================================ //
// 提交成功仅表示事务已被硬件状态机接管；完成状态请通过 iic_async_get_status 查询。
iic_status_enum iic_async_transfer       (iic_index_enum iic_n, uint8 addr,
                                           const uint8 *write_data, uint32 write_len,
                                           uint8 *read_data, uint32 read_len);
void            iic_async_process        (iic_index_enum iic_n);
void            iic_async_process_all    (void);
void            iic_async_process_timed  (iic_index_enum iic_n, uint32 now_tick);
void            iic_async_process_all_timed(uint32 now_tick);
uint8           iic_async_is_busy        (iic_index_enum iic_n);
iic_status_enum iic_async_get_status     (iic_index_enum iic_n);

// ================================ 与逐飞软件 IIC 风格一致的兼容接口 ================================ //
void        iic_write_8bit               (iic_index_enum iic_n, uint8 addr, const uint8 dat);
void        iic_write_8bit_array         (iic_index_enum iic_n, uint8 addr, const uint8 *dat, uint32 len);

void        iic_write_16bit              (iic_index_enum iic_n, uint8 addr, const uint16 dat);
void        iic_write_16bit_array        (iic_index_enum iic_n, uint8 addr, const uint16 *dat, uint32 len);

void        iic_write_8bit_register      (iic_index_enum iic_n, uint8 addr, const uint8 register_name, const uint8 dat);
void        iic_write_8bit_registers     (iic_index_enum iic_n, uint8 addr, const uint8 register_name, const uint8 *dat, uint32 len);

void        iic_write_16bit_register     (iic_index_enum iic_n, uint8 addr, const uint16 register_name, const uint16 dat);
void        iic_write_16bit_registers    (iic_index_enum iic_n, uint8 addr, const uint16 register_name, const uint16 *dat, uint32 len);

uint8       iic_read_8bit                (iic_index_enum iic_n, uint8 addr);
void        iic_read_8bit_array          (iic_index_enum iic_n, uint8 addr, uint8 *dat, uint32 len);

uint16      iic_read_16bit               (iic_index_enum iic_n, uint8 addr);
void        iic_read_16bit_array         (iic_index_enum iic_n, uint8 addr, uint16 *dat, uint32 len);

uint8       iic_read_8bit_register       (iic_index_enum iic_n, uint8 addr, const uint8 register_name);
void        iic_read_8bit_registers      (iic_index_enum iic_n, uint8 addr, const uint8 register_name, uint8 *dat, uint32 len);

uint16      iic_read_16bit_register      (iic_index_enum iic_n, uint8 addr, const uint16 register_name);
void        iic_read_16bit_registers     (iic_index_enum iic_n, uint8 addr, const uint16 register_name, uint16 *dat, uint32 len);

void        iic_transfer_8bit_array      (iic_index_enum iic_n, uint8 addr, const uint8 *write_data, uint32 write_len, uint8 *read_data, uint32 read_len);
void        iic_transfer_16bit_array     (iic_index_enum iic_n, uint8 addr, const uint16 *write_data, uint32 write_len, uint16 *read_data, uint32 read_len);

void        iic_sccb_write_register      (iic_index_enum iic_n, uint8 addr, const uint8 register_name, uint8 dat);
uint8       iic_sccb_read_register       (iic_index_enum iic_n, uint8 addr, const uint8 register_name);

#endif
