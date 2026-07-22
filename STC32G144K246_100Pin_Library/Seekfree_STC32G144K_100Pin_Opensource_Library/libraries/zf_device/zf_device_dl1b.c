/*********************************************************************************************************************
* STC32G144K Opensource Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可声明为译文版本
* 许可声明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2024-08-01        大W            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   SCL                 查看 zf_device_dl1b.h 中 DL1B_SCL_PIN  宏定义
*                   SDA                 查看 zf_device_dl1b.h 中 DL1B_SDA_PIN  宏定义
*                   XS                  查看 zf_device_dl1b.h 中 DL1B_XS_PIN   宏定义
*                   VCC                 5V 电源
*                   GND                 电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"

#include "zf_driver_delay.h"
#include "zf_driver_exti.h"
#include "zf_driver_iic.h"
#include "zf_driver_soft_iic.h"

#include "zf_device_dl1b.h"
#include "zf_device_config.h"
#include "zf_device_type.h"

#pragma warning disable = 177
#pragma warning disable = 183
#pragma warning disable = 209

static uint8 dl1b_init_flag = 0;
uint8 dl1b_finsh_flag = 0;
uint16 dl1b_distance_mm = 8192;
uint8 dl1b_debug_i2c_error = 0;
uint8 dl1b_debug_firmware_status = 0;
uint8 dl1b_debug_model_id = 0;
uint8 dl1b_debug_gpio_status = 0;
uint8 dl1b_debug_range_status = 0;
uint16 dl1b_debug_range_mm = 0;
uint8 dl1b_debug_init_result = 1;
uint8 dl1b_debug_scl_level = 0;
uint8 dl1b_debug_sda_level = 0;
uint8 dl1b_debug_scan_count = 0;
uint8 dl1b_debug_scan_first_address = 0xFF;
static uint8 dl1b_debug_i2c_initialized = 0;

#if (DL1B_USE_INTERFACE==SOFT_IIC) 
	static soft_iic_info_struct dl1b_iic_struct;
	#define dl1b_transfer_8bit_array(tdata, tlen, rdata, rlen)      (dl1b_debug_i2c_error |= soft_iic_transfer_8bit_array_status(&dl1b_iic_struct, (tdata), (tlen), (rdata), (rlen)))

static void dl1b_i2c_init_pins(void)
{
    soft_iic_init(&dl1b_iic_struct, DL1B_DEV_ADDR, DL1B_SOFT_IIC_DELAY, DL1B_SCL_PIN, DL1B_SDA_PIN);

    // 单点主从通信：SCL 由主机推挽驱动，SDA 保持开漏以接收传感器 ACK/数据。
    gpio_init(DL1B_SCL_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(DL1B_SDA_PIN, GPO, 1, GPO_OPEN_DTAIN);
    gpio_set_pull_up(DL1B_SDA_PIN);
}
#elif (DL1B_USE_INTERFACE==HARDWARE_IIC)
static uint8 dl1b_hardware_iic_transfer(const uint8 *write_data, uint32 write_len, uint8 *read_data, uint32 read_len)
{
    iic_transfer_8bit_array(DL1B_IIC, DL1B_DEV_ADDR, write_data, write_len, read_data, read_len);
    return (IIC_SUCCESS == iic_get_last_status(DL1B_IIC)) ? 0 : 1;
}
	#define dl1b_transfer_8bit_array(tdata, tlen, rdata, rlen)      (dl1b_debug_i2c_error |= dl1b_hardware_iic_transfer((tdata), (tlen), (rdata), (rlen)))
#endif

void dl1b_debug_i2c_init(void)
{
#if (DL1B_USE_INTERFACE==SOFT_IIC)
    dl1b_i2c_init_pins();
    dl1b_debug_i2c_initialized = 1;
    dl1b_debug_i2c_error = 0;
    dl1b_debug_scl_level = gpio_get_level(DL1B_SCL_PIN);
    dl1b_debug_sda_level = gpio_get_level(DL1B_SDA_PIN);
#elif (DL1B_USE_INTERFACE==HARDWARE_IIC)
    dl1b_debug_i2c_error = (IIC_SUCCESS == iic_init(DL1B_IIC, DL1B_DEV_ADDR, DL1B_IIC_SPEED, DL1B_SCL_PIN, DL1B_SDA_PIN)) ? 0 : 1;
    dl1b_debug_i2c_initialized = (0 == dl1b_debug_i2c_error) ? 1 : 0;
    dl1b_debug_scl_level = gpio_get_level((gpio_pin_enum)(DL1B_SCL_PIN & 0xFF));
    dl1b_debug_sda_level = gpio_get_level((gpio_pin_enum)(DL1B_SDA_PIN & 0xFF));
#else
    dl1b_debug_i2c_initialized = 0;
#endif
}

void dl1b_debug_i2c_poll(void)
{
#if (DL1B_USE_INTERFACE==SOFT_IIC)
    uint8 data_buffer[3];

    if(0 == dl1b_debug_i2c_initialized)
    {
        dl1b_debug_i2c_init();
    }

    data_buffer[0] = 0x00;
    data_buffer[1] = 0xE5;
    data_buffer[2] = 0;
    dl1b_debug_i2c_error = soft_iic_transfer_8bit_array_status(&dl1b_iic_struct, data_buffer, 2, &data_buffer[2], 1);
    dl1b_debug_firmware_status = data_buffer[2];
    dl1b_debug_scl_level = gpio_get_level(DL1B_SCL_PIN);
    dl1b_debug_sda_level = gpio_get_level(DL1B_SDA_PIN);
#elif (DL1B_USE_INTERFACE==HARDWARE_IIC)
    uint8 data_buffer[3];

    if(0 == dl1b_debug_i2c_initialized)
    {
        dl1b_debug_i2c_init();
    }

    data_buffer[0] = 0x00;
    data_buffer[1] = 0xE5;
    data_buffer[2] = 0;
    dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
    dl1b_debug_firmware_status = data_buffer[2];
    dl1b_debug_scl_level = gpio_get_level((gpio_pin_enum)(DL1B_SCL_PIN & 0xFF));
    dl1b_debug_sda_level = gpio_get_level((gpio_pin_enum)(DL1B_SDA_PIN & 0xFF));
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     返回以毫米为单位的范围读数
// 参数说明     void
// 返回参数     void
// 使用示例     dl1b_get_distance();
// 备注信息     在开始单次射程测量后也调用此函数
//-------------------------------------------------------------------------------------------------------------------
void dl1b_get_distance (void)
{
    if(dl1b_init_flag)
    {
        uint8 data_buffer[3];
        int16 dl1b_distance_temp = 0;
        
        data_buffer[0] = DL1B_GPIO__TIO_HV_STATUS >> 8;
        data_buffer[1] = DL1B_GPIO__TIO_HV_STATUS & 0xFF;
        dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
        dl1b_debug_gpio_status = data_buffer[2];
        
        // DL1B 的 GPIO__TIO_HV_STATUS 返回非零即表示有测量结果。
        // 该器件常见返回值为 0x02，不能只检查 bit0。
        if(data_buffer[2])
        {
            data_buffer[0] = DL1B_SYSTEM__INTERRUPT_CLEAR >> 8;
            data_buffer[1] = DL1B_SYSTEM__INTERRUPT_CLEAR & 0xFF;
            data_buffer[2] = 0x01;
            dl1b_transfer_8bit_array(data_buffer, 3, data_buffer, 0);// clear Interrupt

            data_buffer[0] = DL1B_RESULT__RANGE_STATUS >> 8;
            data_buffer[1] = DL1B_RESULT__RANGE_STATUS & 0xFF;
            dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
            dl1b_debug_range_status = data_buffer[2];
            
            if(0x89 == data_buffer[2])
            {
                data_buffer[0] = DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 >> 8;
                data_buffer[1] = DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 & 0xFF;
                dl1b_transfer_8bit_array(data_buffer, 2, data_buffer, 2);
                dl1b_distance_temp = data_buffer[0];
                dl1b_distance_temp = (dl1b_distance_temp << 8) | data_buffer[1];
                dl1b_debug_range_mm = (uint16)dl1b_distance_temp;
                
                if(dl1b_distance_temp > 4000 || dl1b_distance_temp < 0)
                {
                    dl1b_distance_mm = 8192;
                    dl1b_finsh_flag = 0;
                }
                else
                {
                    dl1b_distance_mm = dl1b_distance_temp;
                    dl1b_finsh_flag = 1;
                }
            }
            else
            {
                dl1b_distance_mm = 8192;
                dl1b_finsh_flag = 0;
            }

        }
        else
        {
            dl1b_distance_mm = 8192;
            dl1b_finsh_flag = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DL1B INT 中断响应处理函数
// 参数说明     void
// 返回参数     void
// 使用示例     dl1b_int_handler();
// 备注信息     本函数需要在 DL1B_INT_PIN 对应的外部中断处理函数中调用
//-------------------------------------------------------------------------------------------------------------------
void dl1b_int_handler (void)
{
#if DL1B_INT_ENABLE
    dl1b_get_distance();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 DL1B
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     dl1b_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 dl1b_init (void)
{
    uint8   return_state    = 0;
    uint8   data_buffer[2 + sizeof(dl1b_config_file)];
    uint16  time_out_count  = 0;

    dl1b_init_flag = 0;
    dl1b_debug_i2c_error = 0;
    dl1b_debug_init_result = 1;
    
#if (DL1B_USE_INTERFACE==SOFT_IIC)        
    dl1b_i2c_init_pins();
    dl1b_debug_scl_level = gpio_get_level(DL1B_SCL_PIN);
    dl1b_debug_sda_level = gpio_get_level(DL1B_SDA_PIN);
    dl1b_debug_scan_count = 0;
    dl1b_debug_scan_first_address = 0xFF;
    system_delay_ms(10);
    {
        uint8 scan_address;
        for(scan_address = 0x08; scan_address <= 0x77; scan_address ++)
        {
            if(0 == soft_iic_probe_7bit_address(&dl1b_iic_struct, scan_address))
            {
                if(0xFF == dl1b_debug_scan_first_address)
                {
                    dl1b_debug_scan_first_address = scan_address;
                }
                dl1b_debug_scan_count ++;
            }
        }
    }
#elif (DL1B_USE_INTERFACE==HARDWARE_IIC)
    if(IIC_SUCCESS != iic_init(DL1B_IIC, DL1B_DEV_ADDR, DL1B_IIC_SPEED, DL1B_SCL_PIN, DL1B_SDA_PIN))
    {
        dl1b_debug_i2c_error = 1;
        dl1b_debug_init_result = 1;
        return 1;
    }
#endif
    
#if	DL1B_XS_ENABLE
    gpio_init(DL1B_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
#endif
    
    do
    {
    
#if	DL1B_XS_ENABLE
        system_delay_ms(50);
        gpio_low(DL1B_XS_PIN);
        system_delay_ms(10);
        gpio_high(DL1B_XS_PIN);
        system_delay_ms(50);
#endif
        
        data_buffer[0] = DL1B_FIRMWARE__SYSTEM_STATUS >> 8;
        data_buffer[1] = DL1B_FIRMWARE__SYSTEM_STATUS & 0xFF;
        dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
        dl1b_debug_scl_level = gpio_get_level(DL1B_SCL_PIN);
        dl1b_debug_sda_level = gpio_get_level(DL1B_SDA_PIN);
        dl1b_debug_firmware_status = data_buffer[2];
        return_state = (0x01 == (data_buffer[2] & 0x01)) ? (0) : (1);
        
        if(1 == return_state)
        {
            break;
        }

        data_buffer[0] = DL1B_IDENTIFICATION__MODEL_ID >> 8;
        data_buffer[1] = DL1B_IDENTIFICATION__MODEL_ID & 0xFF;
        dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
        dl1b_debug_model_id = data_buffer[2];
        if(((uint8)0xEA != data_buffer[2]) || dl1b_debug_i2c_error)
        {
            return_state = 1;
            break;
        }
        
        data_buffer[0] = DL1B_I2C_SLAVE__DEVICE_ADDRESS >> 8;
        data_buffer[1] = DL1B_I2C_SLAVE__DEVICE_ADDRESS & 0xFF;
        
        memcpy(&data_buffer[2], (uint8 *)dl1b_config_file, sizeof(dl1b_config_file));
        dl1b_transfer_8bit_array(data_buffer, 2 + sizeof(dl1b_config_file), data_buffer, 0);
        if(dl1b_debug_i2c_error)
        {
            return_state = 1;
            break;
        }
        
        while(1)
        {
            data_buffer[0] = DL1B_GPIO__TIO_HV_STATUS >> 8;
            data_buffer[1] = DL1B_GPIO__TIO_HV_STATUS & 0xFF;
            dl1b_transfer_8bit_array(data_buffer, 2, &data_buffer[2], 1);
            dl1b_debug_gpio_status = data_buffer[2];

            if(dl1b_debug_i2c_error)
            {
                return_state = 1;
                break;
            }
            
            if(0x00 == (data_buffer[2] & 0x01))
            {
                time_out_count = 0;
                break;
            }
            
            if(DL1B_TIMEOUT_COUNT < time_out_count ++)
            {
                return_state = 1;
                break;
            }
            
            system_delay_ms(1);
        }
        
        if(0 == return_state)
        {
            dl1b_init_flag = 1;
        }
        
    #if DL1B_INT_ENABLE
        exti_init(DL1B_INT_PIN, EXTI_TRIGGER_FALLING);
        dl1b_int_handler();
        dl1b_finsh_flag = 0;
    #endif
        set_tof_type(TOF_DL1B, dl1b_int_handler);
    }
    while(0);
    dl1b_debug_init_result = return_state;
    return return_state;
}
