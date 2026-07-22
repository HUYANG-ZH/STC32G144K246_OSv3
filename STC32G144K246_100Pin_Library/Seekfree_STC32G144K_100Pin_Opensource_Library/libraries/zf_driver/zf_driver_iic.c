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
* 文件名称          zf_driver_iic.c
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
#include "zf_common_debug.h"
#include "zf_driver_iic.h"

#pragma warning disable = 177
#pragma warning disable = 183

#define IIC_CFG_ENABLE                 (0x80U)
#define IIC_CFG_MASTER                 (0x40U)

#define IIC_STATUS_BUSY                (0x80U)
#define IIC_STATUS_IF                  (0x40U)
#define IIC_STATUS_ACK_IN              (0x02U)

#define IIC_CMD_STOP                   (0x06U)
#define IIC_CMD_START_SEND_ACK         (0x09U)
#define IIC_CMD_SEND_ACK_COMBINED      (0x0AU)
#define IIC_CMD_RECV_SEND_ACK          (0x0BU)
#define IIC_CMD_RECV_SEND_NACK         (0x0CU)

#define IIC_DATA_NONE                  (0U)
#define IIC_DATA_8BIT                  (1U)
#define IIC_DATA_16BIT                 (2U)

typedef struct
{
    uint8               initialized;
    volatile uint8      locked;
    uint8               default_addr;
    uint8               pin_group;
    gpio_pin_enum       scl_pin;
    gpio_pin_enum       sda_pin;
    uint16              divider;
    uint32              actual_speed;
    iic_status_enum     last_status;
} iic_info_struct;

static iic_info_struct iic_info[IIC_NUM];

// IIC 总线在硬件初始化和恢复期间需要读取 SDA/SCL 实际电平。
// 该配置属于 IIC 驱动自身，不能调用 GPIO 驱动文件内部的 static 函数。
static void iic_set_digital_input(gpio_pin_enum pin, uint8 enable)
{
    uint8 pin_bit = pin & 0x0F;
    uint8 mask = (uint8)(1 << pin_bit);
    uint8 volatile far *input_enable_register;

    if(pin < IO_P80)
    {
        input_enable_register = (uint8 volatile far *)(0x7efe30 + ((pin >> 4) - 1));
    }
    else
    {
        input_enable_register = (uint8 volatile far *)(0x7ef9e0 + ((pin >> 4) - 9));
    }

    if(enable)
    {
        *input_enable_register |= mask;
    }
    else
    {
        *input_enable_register &= (uint8)(~mask);
    }
}

static uint8 iic_index_is_valid (iic_index_enum iic_n)
{
    return ((IIC_1 == iic_n) || (IIC_2 == iic_n));
}

//-------------------------------------------------------------------------------------------------------------------
// 内部寄存器访问
//-------------------------------------------------------------------------------------------------------------------
static uint8 iic_hw_get_cfg (iic_index_enum iic_n)
{
    return (IIC_1 == iic_n) ? I2CCFG : I2C2CFG;
}

static void iic_hw_set_cfg (iic_index_enum iic_n, uint8 value)
{
    if(IIC_1 == iic_n) I2CCFG = value;
    else               I2C2CFG = value;
}

static uint8 iic_hw_get_status (iic_index_enum iic_n)
{
    return (IIC_1 == iic_n) ? I2CMSST : I2C2MSST;
}

static void iic_hw_clear_if (iic_index_enum iic_n)
{
    if(IIC_1 == iic_n) I2CMSST &= (uint8)(~IIC_STATUS_IF);
    else               I2C2MSST &= (uint8)(~IIC_STATUS_IF);
}

static void iic_hw_set_command (iic_index_enum iic_n, uint8 command)
{
    if(IIC_1 == iic_n) I2CMSCR = command;
    else               I2C2MSCR = command;
}

static void iic_hw_set_tx (iic_index_enum iic_n, uint8 value)
{
    if(IIC_1 == iic_n) I2CTXD = value;
    else               I2C2TXD = value;
}

static uint8 iic_hw_get_rx (iic_index_enum iic_n)
{
    return (IIC_1 == iic_n) ? I2CRXD : I2C2RXD;
}

static void iic_hw_set_aux (iic_index_enum iic_n, uint8 value)
{
    if(IIC_1 == iic_n) I2CMSAUX = value;
    else               I2C2MSAUX = value;
}

static void iic_hw_set_prescaler (iic_index_enum iic_n, uint8 value)
{
    if(IIC_1 == iic_n) I2CPSCR = value;
    else               I2C2PSCR = value;
}

static void iic_set_last_status (iic_index_enum iic_n, iic_status_enum status)
{
    if(iic_index_is_valid(iic_n))
    {
        iic_info[iic_n].last_status = status;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查引脚与 IIC 编号、通道是否严格匹配
//-------------------------------------------------------------------------------------------------------------------
static uint8 iic_check_pin_pair (iic_index_enum iic_n, iic_pin_enum scl_pin, iic_pin_enum sda_pin)
{
    uint16 group_code;

    if((0 == iic_index_is_valid(iic_n)) || (IIC_NULL_PIN == scl_pin) || (IIC_NULL_PIN == sda_pin))
    {
        return 0;
    }

    if((scl_pin & 0xFF00U) != (sda_pin & 0xFF00U))
    {
        return 0;
    }

    group_code = (uint16)(scl_pin & 0xFF00U);

    switch(iic_n)
    {
        case IIC_1:
        {
            switch(group_code)
            {
                case 0x0000: return (((scl_pin & 0xFFU) == IO_P15) && ((sda_pin & 0xFFU) == IO_P14));
                case 0x0100: return (((scl_pin & 0xFFU) == IO_P25) && ((sda_pin & 0xFFU) == IO_P24));
                case 0x0200: return (((scl_pin & 0xFFU) == IO_P77) && ((sda_pin & 0xFFU) == IO_P76));
                case 0x0300: return (((scl_pin & 0xFFU) == IO_P32) && ((sda_pin & 0xFFU) == IO_P33));
                default:     return 0;
            }
        }

        case IIC_2:
        {
            switch(group_code)
            {
                case 0x1000: return (((scl_pin & 0xFFU) == IO_P27) && ((sda_pin & 0xFFU) == IO_P26));
                case 0x1100: return (((scl_pin & 0xFFU) == IO_P37) && ((sda_pin & 0xFFU) == IO_P36));
                case 0x1200: return (((scl_pin & 0xFFU) == IO_P17) && ((sda_pin & 0xFFU) == IO_P16));
                case 0x1300: return (((scl_pin & 0xFFU) == IO_P61) && ((sda_pin & 0xFFU) == IO_P60));
                default:     return 0;
            }
        }

        case IIC_NUM:
        default:
        {
            return 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置复用引脚，且不破坏同一寄存器中的其他外设位
//-------------------------------------------------------------------------------------------------------------------
static void iic_set_pin_mux (iic_index_enum iic_n, uint8 pin_group)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    EAXFR = 1;

    if(IIC_1 == iic_n)
    {
        P_SW2 = (uint8)((P_SW2 & (uint8)(~0x30U)) | ((pin_group & 0x03U) << 4));
    }
    else
    {
        P_SWX4 = (uint8)((P_SWX4 & (uint8)(~0xC0U)) | ((pin_group & 0x03U) << 6));
    }

    EA = ea_backup;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算 14 位 I2CMSSPEED，结果取“不高于请求速度”的最近值
//-------------------------------------------------------------------------------------------------------------------
static uint16 iic_calculate_divider (uint32 speed)
{
    uint32 required_divisor;
    uint32 divider;

    required_divisor = system_clock / speed;
    if(system_clock % speed)
    {
        required_divisor ++;
    }

    if(required_divisor <= 8U)
    {
        divider = 0;
    }
    else
    {
        divider = (required_divisor - 8U + 3U) / 4U;
    }

    if(divider > 0x3FFFU)
    {
        divider = 0x3FFFU;
    }

    return (uint16)divider;
}

static uint32 iic_calculate_actual_speed (uint16 divider)
{
    uint32 denominator = ((uint32)divider * 4U) + 8U;
    return system_clock / denominator;
}

static uint32 iic_calculate_min_speed (void)
{
    uint32 denominator = ((uint32)0x3FFFU * 4U) + 8U;
    uint32 min_speed = system_clock / denominator;

    if(system_clock % denominator)
    {
        min_speed ++;
    }
    return min_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重新装载硬件寄存器。关闭自动发送和中断，采用轮询状态机。
//-------------------------------------------------------------------------------------------------------------------
static void iic_hw_configure (iic_index_enum iic_n)
{
    uint8 cfg;

    iic_hw_set_cfg(iic_n, 0x00);
    iic_hw_set_aux(iic_n, 0x00);
    iic_hw_set_command(iic_n, 0x00);
    iic_hw_clear_if(iic_n);
    iic_hw_set_prescaler(iic_n, (uint8)(iic_info[iic_n].divider >> 6));

    cfg = (uint8)(IIC_CFG_MASTER | (iic_info[iic_n].divider & 0x3FU));
    iic_hw_set_cfg(iic_n, cfg);
    iic_hw_set_cfg(iic_n, (uint8)(cfg | IIC_CFG_ENABLE));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     尝试获得非重入锁。只在检查并置位的极短区间关闭全局中断。
//-------------------------------------------------------------------------------------------------------------------
static uint8 iic_try_lock (iic_index_enum iic_n)
{
    uint8 ea_backup;
    uint8 result = 0;

    ea_backup = EA;
    EA = 0;
    if(0 == iic_info[iic_n].locked)
    {
        iic_info[iic_n].locked = 1;
        result = 1;
    }
    EA = ea_backup;

    return result;
}

static void iic_unlock (iic_index_enum iic_n)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    iic_info[iic_n].locked = 0;
    EA = ea_backup;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     总线恢复专用短延时。使用 CPU 空循环，不占用工程中的 Timer11 或其他硬件定时器。
// 备注信息     总线恢复只要求脉冲不短于目标值，不参与正常 IIC 位时序；实际延时允许略长。
//-------------------------------------------------------------------------------------------------------------------
static void iic_recovery_delay_us (uint16 period_us)
{
    volatile uint32 delay_count;
    uint32 loops_per_us;

    loops_per_us = system_clock / 1000000UL;
    if(0U == loops_per_us)
    {
        loops_per_us = 1U;
    }

    if(0U == period_us)
    {
        period_us = 1U;
    }

    delay_count = loops_per_us * (uint32)period_us;
    while(delay_count)
    {
        delay_count --;
        _nop_();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一个硬件命令并等待完成；任何路径都有限时，不存在死循环。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_execute_command (iic_index_enum iic_n, uint8 command)
{
    uint32 timeout = IIC_TIMEOUT_COUNT;

    iic_hw_clear_if(iic_n);
    iic_hw_set_command(iic_n, command);

    while(0 == (iic_hw_get_status(iic_n) & IIC_STATUS_IF))
    {
        if(0 == timeout --)
        {
            return IIC_ERROR_TIMEOUT;
        }
    }

    iic_hw_clear_if(iic_n);
    return IIC_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPIO 层总线恢复：最多 9 个 SCL 脉冲并生成 STOP。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_recover_bus_internal (iic_index_enum iic_n)
{
    uint8 pulse_count;
    uint16 wait_count;
    gpio_pin_enum scl_pin = iic_info[iic_n].scl_pin;
    gpio_pin_enum sda_pin = iic_info[iic_n].sda_pin;

    iic_hw_set_cfg(iic_n, (uint8)(iic_hw_get_cfg(iic_n) & (uint8)(~IIC_CFG_ENABLE)));

    gpio_init(scl_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(sda_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    iic_set_digital_input(scl_pin, GPIO_DIGITAL_IN_ENABLE);
    iic_set_digital_input(sda_pin, GPIO_DIGITAL_IN_ENABLE);
    iic_recovery_delay_us(IIC_RECOVERY_PULSE_US);

    // SCL 被目标器件永久拉低时，主机无法恢复，只能明确返回错误。
    wait_count = 1000U;
    while((0 == gpio_get_level(scl_pin)) && wait_count --)
    {
        iic_recovery_delay_us(1);
    }
    if(0 == gpio_get_level(scl_pin))
    {
        iic_hw_configure(iic_n);
        return IIC_ERROR_BUS_STUCK;
    }

    for(pulse_count = 0; (pulse_count < 9U) && (0 == gpio_get_level(sda_pin)); pulse_count ++)
    {
        gpio_low(scl_pin);
        iic_recovery_delay_us(IIC_RECOVERY_PULSE_US);
        gpio_high(scl_pin);

        wait_count = 1000U;
        while((0 == gpio_get_level(scl_pin)) && wait_count --)
        {
            iic_recovery_delay_us(1);
        }
        if(0 == gpio_get_level(scl_pin))
        {
            iic_hw_configure(iic_n);
            return IIC_ERROR_BUS_STUCK;
        }
        iic_recovery_delay_us(IIC_RECOVERY_PULSE_US);
    }

    // 生成一个标准 STOP：SDA 低 -> SCL 高 -> SDA 高。
    gpio_low(sda_pin);
    iic_recovery_delay_us(IIC_RECOVERY_PULSE_US);
    gpio_high(scl_pin);
    iic_recovery_delay_us(IIC_RECOVERY_PULSE_US);
    gpio_high(sda_pin);
    iic_recovery_delay_us(IIC_RECOVERY_PULSE_US);

    iic_set_pin_mux(iic_n, iic_info[iic_n].pin_group);
    iic_hw_configure(iic_n);

    if((0 == gpio_get_level(scl_pin)) || (0 == gpio_get_level(sda_pin)))
    {
        return IIC_ERROR_BUS_STUCK;
    }

    return IIC_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待总线空闲，超时后进行一次恢复。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_prepare_transaction (iic_index_enum iic_n)
{
    uint32 timeout = IIC_TIMEOUT_COUNT;

    // I2C1/I2C2 寄存器位于 XFR；防止用户代码曾关闭 EAXFR 后访问失效。
    EAXFR = 1;

    while(iic_hw_get_status(iic_n) & IIC_STATUS_BUSY)
    {
        if(0 == timeout --)
        {
            return iic_recover_bus_internal(iic_n);
        }
    }

    if((0 == gpio_get_level(iic_info[iic_n].scl_pin)) ||
       (0 == gpio_get_level(iic_info[iic_n].sda_pin)))
    {
        return iic_recover_bus_internal(iic_n);
    }

    return IIC_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送 START/Repeated START、地址和读写位，并检查目标应答。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_start_address (iic_index_enum iic_n, uint8 addr, uint8 read)
{
    iic_status_enum status;

    iic_hw_set_tx(iic_n, (uint8)(((uint32)addr << 1) | (read ? 1U : 0U)));
    status = iic_execute_command(iic_n, IIC_CMD_START_SEND_ACK);
    if(IIC_SUCCESS != status)
    {
        return status;
    }

    return (iic_hw_get_status(iic_n) & IIC_STATUS_ACK_IN) ? IIC_ERROR_NACK : IIC_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送一个字节并检查 ACK。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_send_byte (iic_index_enum iic_n, uint8 value)
{
    iic_status_enum status;

    iic_hw_set_tx(iic_n, value);
    status = iic_execute_command(iic_n, IIC_CMD_SEND_ACK_COMBINED);
    if(IIC_SUCCESS != status)
    {
        return status;
    }

    return (iic_hw_get_status(iic_n) & IIC_STATUS_ACK_IN) ? IIC_ERROR_NACK : IIC_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收一个字节。最后一字节发送 NACK，其余发送 ACK。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_receive_byte (iic_index_enum iic_n, uint8 *value, uint8 last_byte)
{
    iic_status_enum status;

    status = iic_execute_command(iic_n, last_byte ? IIC_CMD_RECV_SEND_NACK : IIC_CMD_RECV_SEND_ACK);
    if(IIC_SUCCESS == status)
    {
        *value = iic_hw_get_rx(iic_n);
    }
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     尽最大努力发送 STOP。失败时复位模块并恢复总线，防止后续调用永久卡死。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_stop_transaction (iic_index_enum iic_n)
{
    iic_status_enum status;

    status = iic_execute_command(iic_n, IIC_CMD_STOP);
    if(IIC_SUCCESS != status)
    {
        iic_recover_bus_internal(iic_n);
        return status;
    }

    return IIC_SUCCESS;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     统一事务引擎。支持 0/1/2 字节前缀、8/16 位数据、Repeated START 与 SCCB STOP。
//-------------------------------------------------------------------------------------------------------------------
static iic_status_enum iic_transaction_engine (
    iic_index_enum iic_n,
    uint8 addr,
    const uint8 *prefix,
    uint8 prefix_len,
    const void *write_data,
    uint32 write_len,
    uint8 write_width,
    void *read_data,
    uint32 read_len,
    uint8 read_width,
    uint8 stop_between)
{
    iic_status_enum status = IIC_SUCCESS;
    iic_status_enum stop_status;
    uint8 started = 0;
    uint8 value;
    uint8 index;
    uint32 count;
    const uint8 *write8;
    const uint16 *write16;
    uint8 *read8;
    uint16 *read16;

    if((0 == iic_index_is_valid(iic_n)) || (addr > 0x7FU) || (prefix_len > 2U) ||
       ((write_width != IIC_DATA_NONE) && (write_width != IIC_DATA_8BIT) && (write_width != IIC_DATA_16BIT)) ||
       ((read_width != IIC_DATA_NONE) && (read_width != IIC_DATA_8BIT) && (read_width != IIC_DATA_16BIT)) ||
       ((prefix_len > 0U) && (NULL == prefix)) ||
       ((write_len > 0U) && ((NULL == write_data) || (IIC_DATA_NONE == write_width))) ||
       ((read_len > 0U) && ((NULL == read_data) || (IIC_DATA_NONE == read_width))))
    {
        iic_set_last_status(iic_n, IIC_ERROR_INVALID_PARAMETER);
        return IIC_ERROR_INVALID_PARAMETER;
    }

    if(0 == iic_info[iic_n].initialized)
    {
        iic_set_last_status(iic_n, IIC_ERROR_NOT_INITIALIZED);
        return IIC_ERROR_NOT_INITIALIZED;
    }

    if((0U == prefix_len) && (0U == write_len) && (0U == read_len))
    {
        iic_set_last_status(iic_n, IIC_SUCCESS);
        return IIC_SUCCESS;
    }

    if(0 == iic_try_lock(iic_n))
    {
        iic_set_last_status(iic_n, IIC_ERROR_BUSY);
        return IIC_ERROR_BUSY;
    }

    status = iic_prepare_transaction(iic_n);
    if(IIC_SUCCESS != status)
    {
        goto transaction_exit;
    }

    // 有前缀或写数据时，先开启写方向事务。
    if((prefix_len > 0U) || (write_len > 0U))
    {
        status = iic_start_address(iic_n, addr, 0);
        started = 1;
        if(IIC_SUCCESS != status) goto transaction_fail;

        for(index = 0; index < prefix_len; index ++)
        {
            status = iic_send_byte(iic_n, prefix[index]);
            if(IIC_SUCCESS != status) goto transaction_fail;
        }

        if(IIC_DATA_8BIT == write_width)
        {
            write8 = (const uint8 *)write_data;
            for(count = 0; count < write_len; count ++)
            {
                status = iic_send_byte(iic_n, write8[count]);
                if(IIC_SUCCESS != status) goto transaction_fail;
            }
        }
        else if(IIC_DATA_16BIT == write_width)
        {
            write16 = (const uint16 *)write_data;
            for(count = 0; count < write_len; count ++)
            {
                status = iic_send_byte(iic_n, (uint8)(write16[count] >> 8));
                if(IIC_SUCCESS != status) goto transaction_fail;
                status = iic_send_byte(iic_n, (uint8)write16[count]);
                if(IIC_SUCCESS != status) goto transaction_fail;
            }
        }
    }

    if(read_len > 0U)
    {
        if(stop_between && started)
        {
            status = iic_stop_transaction(iic_n);
            if(IIC_SUCCESS != status) goto transaction_exit;

            status = iic_prepare_transaction(iic_n);
            if(IIC_SUCCESS != status) goto transaction_exit;
        }

        status = iic_start_address(iic_n, addr, 1);
        started = 1;
        if(IIC_SUCCESS != status) goto transaction_fail;

        if(IIC_DATA_8BIT == read_width)
        {
            read8 = (uint8 *)read_data;
            for(count = 0; count < read_len; count ++)
            {
                status = iic_receive_byte(iic_n, &read8[count], (uint8)((count + 1U) == read_len));
                if(IIC_SUCCESS != status) goto transaction_fail;
            }
        }
        else if(IIC_DATA_16BIT == read_width)
        {
            read16 = (uint16 *)read_data;
            for(count = 0; count < read_len; count ++)
            {
                status = iic_receive_byte(iic_n, &value, 0);
                if(IIC_SUCCESS != status) goto transaction_fail;
                read16[count] = (uint16)((uint16)value << 8);

                status = iic_receive_byte(iic_n, &value, (uint8)((count + 1U) == read_len));
                if(IIC_SUCCESS != status) goto transaction_fail;
                read16[count] |= value;
            }
        }
    }

    if(started)
    {
        status = iic_stop_transaction(iic_n);
    }
    goto transaction_exit;

transaction_fail:
    if(started)
    {
        if(IIC_ERROR_TIMEOUT == status)
        {
            // 硬件命令已卡住时不再追加 STOP 命令，直接关闭模块并恢复总线。
            iic_recover_bus_internal(iic_n);
        }
        else
        {
            stop_status = iic_stop_transaction(iic_n);
            if(IIC_SUCCESS != stop_status)
            {
                status = stop_status;
            }
        }
    }

transaction_exit:
    iic_set_last_status(iic_n, status);
    iic_unlock(iic_n);
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化硬件 IIC，默认主机、7 位地址、轮询模式
//-------------------------------------------------------------------------------------------------------------------
iic_status_enum iic_init (iic_index_enum iic_n, uint8 addr, uint32 speed, iic_pin_enum scl_pin, iic_pin_enum sda_pin)
{
    uint16 divider;
    uint8 pin_group;

    if((0 == iic_index_is_valid(iic_n)) || (addr > 0x7FU) || (0U == speed) ||
       (0U == system_clock) ||
       (speed < iic_calculate_min_speed()) ||
       (0 == iic_check_pin_pair(iic_n, scl_pin, sda_pin)))
    {
        iic_set_last_status(iic_n, IIC_ERROR_INVALID_PARAMETER);
        return IIC_ERROR_INVALID_PARAMETER;
    }

    if(0 == iic_try_lock(iic_n))
    {
        iic_set_last_status(iic_n, IIC_ERROR_BUSY);
        return IIC_ERROR_BUSY;
    }

    divider = iic_calculate_divider(speed);
    pin_group = (uint8)((scl_pin >> 8) & 0x03U);

    iic_info[iic_n].initialized  = 0;
    iic_info[iic_n].default_addr = addr;
    iic_info[iic_n].pin_group    = pin_group;
    iic_info[iic_n].scl_pin      = (gpio_pin_enum)(scl_pin & 0xFFU);
    iic_info[iic_n].sda_pin      = (gpio_pin_enum)(sda_pin & 0xFFU);
    iic_info[iic_n].divider      = divider;
    iic_info[iic_n].actual_speed = iic_calculate_actual_speed(divider);
    iic_info[iic_n].last_status  = IIC_SUCCESS;

    gpio_init(iic_info[iic_n].scl_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    gpio_init(iic_info[iic_n].sda_pin, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);
    iic_set_digital_input(iic_info[iic_n].scl_pin, GPIO_DIGITAL_IN_ENABLE);
    iic_set_digital_input(iic_info[iic_n].sda_pin, GPIO_DIGITAL_IN_ENABLE);
    iic_set_pin_mux(iic_n, pin_group);
    iic_hw_configure(iic_n);
    iic_info[iic_n].initialized = 1;

    // 上电时若目标器件把 SDA 留在低电平，主动进行一次恢复。
    if((0 == gpio_get_level(iic_info[iic_n].scl_pin)) ||
       (0 == gpio_get_level(iic_info[iic_n].sda_pin)))
    {
        iic_info[iic_n].last_status = iic_recover_bus_internal(iic_n);
    }

    iic_unlock(iic_n);
    return iic_info[iic_n].last_status;
}

//-------------------------------------------------------------------------------------------------------------------
// 带状态返回的底层接口
//-------------------------------------------------------------------------------------------------------------------
iic_status_enum iic_write (iic_index_enum iic_n, uint8 addr, const uint8 *buffer, uint32 len)
{
    return iic_transaction_engine(iic_n, addr, NULL, 0, buffer, len, IIC_DATA_8BIT, NULL, 0, IIC_DATA_NONE, 0);
}

iic_status_enum iic_read (iic_index_enum iic_n, uint8 addr, uint8 *buffer, uint32 len)
{
    return iic_transaction_engine(iic_n, addr, NULL, 0, NULL, 0, IIC_DATA_NONE, buffer, len, IIC_DATA_8BIT, 0);
}

iic_status_enum iic_transfer (iic_index_enum iic_n, uint8 addr, const uint8 *write_data, uint32 write_len, uint8 *read_data, uint32 read_len)
{
    return iic_transaction_engine(iic_n, addr, NULL, 0, write_data, write_len, IIC_DATA_8BIT, read_data, read_len, IIC_DATA_8BIT, 0);
}

iic_status_enum iic_recover_bus (iic_index_enum iic_n)
{
    iic_status_enum status;

    if(0 == iic_index_is_valid(iic_n)) return IIC_ERROR_INVALID_PARAMETER;
    if(0 == iic_info[iic_n].initialized)
    {
        iic_set_last_status(iic_n, IIC_ERROR_NOT_INITIALIZED);
        return IIC_ERROR_NOT_INITIALIZED;
    }
    if(0 == iic_try_lock(iic_n))
    {
        iic_set_last_status(iic_n, IIC_ERROR_BUSY);
        return IIC_ERROR_BUSY;
    }

    status = iic_recover_bus_internal(iic_n);
    iic_set_last_status(iic_n, status);
    iic_unlock(iic_n);
    return status;
}

iic_status_enum iic_get_last_status (iic_index_enum iic_n)
{
    return iic_index_is_valid(iic_n) ? iic_info[iic_n].last_status : IIC_ERROR_INVALID_PARAMETER;
}

uint32 iic_get_actual_speed (iic_index_enum iic_n)
{
    return (iic_index_is_valid(iic_n) && iic_info[iic_n].initialized) ? iic_info[iic_n].actual_speed : 0U;
}

void iic_deinit (iic_index_enum iic_n)
{
    if(0 == iic_index_is_valid(iic_n))
    {
        return;
    }

    if(0 == iic_try_lock(iic_n))
    {
        iic_set_last_status(iic_n, IIC_ERROR_BUSY);
        return;
    }

    EAXFR = 1;
    iic_hw_set_cfg(iic_n, 0x00);
    iic_info[iic_n].initialized = 0;
    iic_info[iic_n].last_status = IIC_ERROR_NOT_INITIALIZED;
    iic_unlock(iic_n);
}

//-------------------------------------------------------------------------------------------------------------------
// 与软件 IIC 风格一致的兼容封装
//-------------------------------------------------------------------------------------------------------------------
void iic_write_8bit (iic_index_enum iic_n, uint8 addr, const uint8 dat)
{
    iic_write(iic_n, addr, &dat, 1);
}

void iic_write_8bit_array (iic_index_enum iic_n, uint8 addr, const uint8 *dat, uint32 len)
{
    iic_write(iic_n, addr, dat, len);
}

void iic_write_16bit (iic_index_enum iic_n, uint8 addr, const uint16 dat)
{
    iic_transaction_engine(iic_n, addr, NULL, 0, &dat, 1, IIC_DATA_16BIT, NULL, 0, IIC_DATA_NONE, 0);
}

void iic_write_16bit_array (iic_index_enum iic_n, uint8 addr, const uint16 *dat, uint32 len)
{
    iic_transaction_engine(iic_n, addr, NULL, 0, dat, len, IIC_DATA_16BIT, NULL, 0, IIC_DATA_NONE, 0);
}

void iic_write_8bit_register (iic_index_enum iic_n, uint8 addr, const uint8 register_name, const uint8 dat)
{
    iic_transaction_engine(iic_n, addr, &register_name, 1, &dat, 1, IIC_DATA_8BIT, NULL, 0, IIC_DATA_NONE, 0);
}

void iic_write_8bit_registers (iic_index_enum iic_n, uint8 addr, const uint8 register_name, const uint8 *dat, uint32 len)
{
    iic_transaction_engine(iic_n, addr, &register_name, 1, dat, len, IIC_DATA_8BIT, NULL, 0, IIC_DATA_NONE, 0);
}

void iic_write_16bit_register (iic_index_enum iic_n, uint8 addr, const uint16 register_name, const uint16 dat)
{
    uint8 prefix[2];
    prefix[0] = (uint8)(register_name >> 8);
    prefix[1] = (uint8)register_name;
    iic_transaction_engine(iic_n, addr, prefix, 2, &dat, 1, IIC_DATA_16BIT, NULL, 0, IIC_DATA_NONE, 0);
}

void iic_write_16bit_registers (iic_index_enum iic_n, uint8 addr, const uint16 register_name, const uint16 *dat, uint32 len)
{
    uint8 prefix[2];
    prefix[0] = (uint8)(register_name >> 8);
    prefix[1] = (uint8)register_name;
    iic_transaction_engine(iic_n, addr, prefix, 2, dat, len, IIC_DATA_16BIT, NULL, 0, IIC_DATA_NONE, 0);
}

uint8 iic_read_8bit (iic_index_enum iic_n, uint8 addr)
{
    uint8 dat = 0;
    iic_read(iic_n, addr, &dat, 1);
    return dat;
}

void iic_read_8bit_array (iic_index_enum iic_n, uint8 addr, uint8 *dat, uint32 len)
{
    iic_read(iic_n, addr, dat, len);
}

uint16 iic_read_16bit (iic_index_enum iic_n, uint8 addr)
{
    uint16 dat = 0;
    iic_transaction_engine(iic_n, addr, NULL, 0, NULL, 0, IIC_DATA_NONE, &dat, 1, IIC_DATA_16BIT, 0);
    return dat;
}

void iic_read_16bit_array (iic_index_enum iic_n, uint8 addr, uint16 *dat, uint32 len)
{
    iic_transaction_engine(iic_n, addr, NULL, 0, NULL, 0, IIC_DATA_NONE, dat, len, IIC_DATA_16BIT, 0);
}

uint8 iic_read_8bit_register (iic_index_enum iic_n, uint8 addr, const uint8 register_name)
{
    uint8 dat = 0;
    iic_transaction_engine(iic_n, addr, &register_name, 1, NULL, 0, IIC_DATA_NONE, &dat, 1, IIC_DATA_8BIT, 0);
    return dat;
}

void iic_read_8bit_registers (iic_index_enum iic_n, uint8 addr, const uint8 register_name, uint8 *dat, uint32 len)
{
    iic_transaction_engine(iic_n, addr, &register_name, 1, NULL, 0, IIC_DATA_NONE, dat, len, IIC_DATA_8BIT, 0);
}

uint16 iic_read_16bit_register (iic_index_enum iic_n, uint8 addr, const uint16 register_name)
{
    uint8 prefix[2];
    uint16 dat = 0;
    prefix[0] = (uint8)(register_name >> 8);
    prefix[1] = (uint8)register_name;
    iic_transaction_engine(iic_n, addr, prefix, 2, NULL, 0, IIC_DATA_NONE, &dat, 1, IIC_DATA_16BIT, 0);
    return dat;
}

void iic_read_16bit_registers (iic_index_enum iic_n, uint8 addr, const uint16 register_name, uint16 *dat, uint32 len)
{
    uint8 prefix[2];
    prefix[0] = (uint8)(register_name >> 8);
    prefix[1] = (uint8)register_name;
    iic_transaction_engine(iic_n, addr, prefix, 2, NULL, 0, IIC_DATA_NONE, dat, len, IIC_DATA_16BIT, 0);
}

void iic_transfer_8bit_array (iic_index_enum iic_n, uint8 addr, const uint8 *write_data, uint32 write_len, uint8 *read_data, uint32 read_len)
{
    iic_transfer(iic_n, addr, write_data, write_len, read_data, read_len);
}

void iic_transfer_16bit_array (iic_index_enum iic_n, uint8 addr, const uint16 *write_data, uint32 write_len, uint16 *read_data, uint32 read_len)
{
    iic_transaction_engine(iic_n, addr, NULL, 0, write_data, write_len, IIC_DATA_16BIT, read_data, read_len, IIC_DATA_16BIT, 0);
}

void iic_sccb_write_register (iic_index_enum iic_n, uint8 addr, const uint8 register_name, uint8 dat)
{
    iic_write_8bit_register(iic_n, addr, register_name, dat);
}

uint8 iic_sccb_read_register (iic_index_enum iic_n, uint8 addr, const uint8 register_name)
{
    uint8 dat = 0;
    iic_transaction_engine(iic_n, addr, &register_name, 1, NULL, 0, IIC_DATA_NONE, &dat, 1, IIC_DATA_8BIT, 1);
    return dat;
}
