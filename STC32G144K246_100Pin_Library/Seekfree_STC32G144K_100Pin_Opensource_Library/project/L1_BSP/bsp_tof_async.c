#include "zf_common_headfile.h"
#include "bsp_tof.h"
#include "bsp_tof_async.h"
#include "service_timetick.h"
#include "zf_device_config.h"
#include "zf_device_type.h"
#include "zf_driver_iic.h"

#if (BSP_TOF_DRIVER == BSP_TOF_DRIVER_DL1A)
    #include "zf_device_dl1a.h"
#elif (BSP_TOF_DRIVER == BSP_TOF_DRIVER_DL1B)
    #include "zf_device_dl1b.h"
#else
    #error "Unsupported BSP_TOF_DRIVER"
#endif

/* ------------------------------------------------------------------ */
/*  DL1B 异步 IIC 状态机                                               */
/* ------------------------------------------------------------------ */
#if (BSP_TOF_DRIVER == BSP_TOF_DRIVER_DL1B)

#define BSP_TOF_CONFIG_BYTES                (135U)
#define BSP_TOF_TRANSFER_BYTES              (BSP_TOF_CONFIG_BYTES + 2U)
#define BSP_TOF_XS_HIGH_WAIT_TICK           (500UL)
#define BSP_TOF_XS_LOW_WAIT_TICK            (100UL)
#define BSP_TOF_XS_BOOT_WAIT_TICK           (500UL)
#define BSP_TOF_READY_POLL_TICK             (10UL)
#define BSP_TOF_READY_MAX_POLLS              (1000U)
#define BSP_TOF_RETRY_WAIT_TICK              (10000UL)
#define BSP_TOF_SUBMIT_BUSY_TIMEOUT_TICK     (100UL)

typedef enum
{
    BSP_TOF_STATE_IDLE = 0,
    BSP_TOF_STATE_XS_HIGH_WAIT,
    BSP_TOF_STATE_XS_LOW_WAIT,
    BSP_TOF_STATE_XS_BOOT_WAIT,
    BSP_TOF_STATE_IIC_INIT,
    BSP_TOF_STATE_FIRMWARE_SUBMIT,
    BSP_TOF_STATE_FIRMWARE_WAIT,
    BSP_TOF_STATE_MODEL_SUBMIT,
    BSP_TOF_STATE_MODEL_WAIT,
    BSP_TOF_STATE_CONFIG_SUBMIT,
    BSP_TOF_STATE_CONFIG_WAIT,
    BSP_TOF_STATE_READY_POLL_DELAY,
    BSP_TOF_STATE_READY_POLL_SUBMIT,
    BSP_TOF_STATE_READY_POLL_WAIT,
    BSP_TOF_STATE_READY,
    BSP_TOF_STATE_SAMPLE_GPIO_SUBMIT,
    BSP_TOF_STATE_SAMPLE_GPIO_WAIT,
    BSP_TOF_STATE_SAMPLE_CLEAR_SUBMIT,
    BSP_TOF_STATE_SAMPLE_CLEAR_WAIT,
    BSP_TOF_STATE_SAMPLE_STATUS_SUBMIT,
    BSP_TOF_STATE_SAMPLE_STATUS_WAIT,
    BSP_TOF_STATE_SAMPLE_DISTANCE_SUBMIT,
    BSP_TOF_STATE_SAMPLE_DISTANCE_WAIT,
    BSP_TOF_STATE_ERROR,
} bsp_tof_state_enum;

static uint8 xdata tof_transfer_buffer[BSP_TOF_TRANSFER_BYTES];
static uint8 xdata tof_read_buffer[2];
static uint16 tof_distance_mm = BSP_TOF_INVALID_DISTANCE_MM;
static uint32 tof_deadline_tick = 0UL;
static uint16 tof_ready_poll_count = 0U;
static uint32 tof_retry_deadline_tick = 0UL;
static uint32 tof_submit_busy_deadline_tick = 0UL;
static uint8 tof_initialized = 0U;
static uint8 tof_ready = 0U;
static uint8 tof_sample_requested = 0U;
static uint8 tof_last_error = 0U;
static uint8 tof_submit_busy_pending = 0U;
static bsp_tof_state_enum tof_state = BSP_TOF_STATE_IDLE;

static uint8 bsp_tof_deadline_reached(uint32 now)
{
    return ((uint32)(now - tof_deadline_tick) < 0x80000000UL) ? 1U : 0U;
}

static uint8 bsp_tof_retry_deadline_reached(uint32 now)
{
    return ((uint32)(now - tof_retry_deadline_tick) < 0x80000000UL) ? 1U : 0U;
}

static void bsp_tof_fail(iic_status_enum status)
{
    tof_last_error = (uint8)status;
    tof_distance_mm = BSP_TOF_INVALID_DISTANCE_MM;
    tof_ready = 0U;
    tof_sample_requested = 0U;
    tof_submit_busy_pending = 0U;
    tof_retry_deadline_tick = service_timetick_what() + BSP_TOF_RETRY_WAIT_TICK;
    tof_state = BSP_TOF_STATE_ERROR;
}

static uint8 bsp_tof_submit(uint16 reg, uint8 write_len, uint8 read_len)
{
    iic_status_enum status;
    uint32 now;

    tof_transfer_buffer[0] = (uint8)(reg >> 8);
    tof_transfer_buffer[1] = (uint8)reg;
    status = iic_async_transfer(DL1B_IIC, DL1B_DEV_ADDR,
            tof_transfer_buffer, (uint32)(write_len + 2U), tof_read_buffer, read_len);
    if(IIC_SUCCESS == status)
    {
        tof_submit_busy_pending = 0U;
        return 1U;
    }
    if(IIC_ERROR_BUSY == status)
    {
        now = service_timetick_what();
        if(0U == tof_submit_busy_pending)
        {
            tof_submit_busy_pending = 1U;
            tof_submit_busy_deadline_tick = now + BSP_TOF_SUBMIT_BUSY_TIMEOUT_TICK;
            return 0U;
        }
        if((uint32)(now - tof_submit_busy_deadline_tick) < 0x80000000UL)
        {
            bsp_tof_fail(IIC_ERROR_TIMEOUT);
            return 2U;
        }
        return 0U;
    }

    bsp_tof_fail(status);
    return 2U;
}

static uint8 bsp_tof_transfer_succeeded(void)
{
    iic_status_enum status;

    if(0U != iic_async_is_busy(DL1B_IIC))
    {
        return 0U;
    }
    status = iic_async_get_status(DL1B_IIC);
    if(IIC_SUCCESS != status)
    {
        bsp_tof_fail(status);
        return 0U;
    }
    return 1U;
}

uint8 bsp_tof_async_init(void)
{
    uint8 index;

    tof_initialized = 1U;

    for(index = 0U; index < BSP_TOF_CONFIG_BYTES; index++)
    {
        tof_transfer_buffer[index + 2U] = dl1b_config_file[index];
    }
    tof_read_buffer[0] = 0U;
    tof_read_buffer[1] = 0U;
    tof_distance_mm = BSP_TOF_INVALID_DISTANCE_MM;
    tof_ready_poll_count = 0U;
    tof_ready = 0U;
    tof_sample_requested = 0U;
    tof_last_error = IIC_SUCCESS;
    tof_retry_deadline_tick = 0UL;
    tof_submit_busy_deadline_tick = 0UL;
    tof_submit_busy_pending = 0U;
    set_tof_type(TOF_DL1B, 0);

#if DL1B_XS_ENABLE
    gpio_init(DL1B_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
#endif
    tof_deadline_tick = service_timetick_what() + BSP_TOF_XS_HIGH_WAIT_TICK;
    tof_state = BSP_TOF_STATE_XS_HIGH_WAIT;
    return 1U;
}

void bsp_tof_async_process(void)
{
    uint8 submit_result;
    uint32 now;
    iic_status_enum status;

    if(0U == tof_initialized)
    {
        return;
    }

    now = service_timetick_what();
    switch(tof_state)
    {
        case BSP_TOF_STATE_XS_HIGH_WAIT:
            if(0U != bsp_tof_deadline_reached(now))
            {
#if DL1B_XS_ENABLE
                gpio_low(DL1B_XS_PIN);
#endif
                tof_deadline_tick = now + BSP_TOF_XS_LOW_WAIT_TICK;
                tof_state = BSP_TOF_STATE_XS_LOW_WAIT;
            }
            break;

        case BSP_TOF_STATE_XS_LOW_WAIT:
            if(0U != bsp_tof_deadline_reached(now))
            {
#if DL1B_XS_ENABLE
                gpio_high(DL1B_XS_PIN);
#endif
                tof_deadline_tick = now + BSP_TOF_XS_BOOT_WAIT_TICK;
                tof_state = BSP_TOF_STATE_XS_BOOT_WAIT;
            }
            break;

        case BSP_TOF_STATE_XS_BOOT_WAIT:
            if(0U != bsp_tof_deadline_reached(now))
            {
                tof_state = BSP_TOF_STATE_IIC_INIT;
            }
            break;

        case BSP_TOF_STATE_IIC_INIT:
            status = iic_init(DL1B_IIC, DL1B_DEV_ADDR, DL1B_IIC_SPEED,
                    DL1B_SCL_PIN, DL1B_SDA_PIN);
            if(IIC_SUCCESS != status)
            {
                bsp_tof_fail(status);
            }
            else
            {
                tof_state = BSP_TOF_STATE_FIRMWARE_SUBMIT;
            }
            break;

        case BSP_TOF_STATE_FIRMWARE_SUBMIT:
            submit_result = bsp_tof_submit(DL1B_FIRMWARE__SYSTEM_STATUS, 0U, 1U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_FIRMWARE_WAIT;
            }
            break;

        case BSP_TOF_STATE_FIRMWARE_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                if(0U == (tof_read_buffer[0] & 0x01U))
                {
                    bsp_tof_fail(IIC_ERROR_NACK);
                }
                else
                {
                    tof_state = BSP_TOF_STATE_MODEL_SUBMIT;
                }
            }
            break;

        case BSP_TOF_STATE_MODEL_SUBMIT:
            submit_result = bsp_tof_submit(DL1B_IDENTIFICATION__MODEL_ID, 0U, 1U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_MODEL_WAIT;
            }
            break;

        case BSP_TOF_STATE_MODEL_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                if(0xEAU != tof_read_buffer[0])
                {
                    bsp_tof_fail(IIC_ERROR_NACK);
                }
                else
                {
                    tof_state = BSP_TOF_STATE_CONFIG_SUBMIT;
                }
            }
            break;

        case BSP_TOF_STATE_CONFIG_SUBMIT:
            submit_result = bsp_tof_submit(DL1B_I2C_SLAVE__DEVICE_ADDRESS, BSP_TOF_CONFIG_BYTES, 0U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_CONFIG_WAIT;
            }
            break;

        case BSP_TOF_STATE_CONFIG_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                tof_ready_poll_count = 0U;
                tof_deadline_tick = now;
                tof_state = BSP_TOF_STATE_READY_POLL_DELAY;
            }
            break;

        case BSP_TOF_STATE_READY_POLL_DELAY:
            if(0U != bsp_tof_deadline_reached(now))
            {
                tof_state = BSP_TOF_STATE_READY_POLL_SUBMIT;
            }
            break;

        case BSP_TOF_STATE_READY_POLL_SUBMIT:
            submit_result = bsp_tof_submit(DL1B_GPIO__TIO_HV_STATUS, 0U, 1U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_READY_POLL_WAIT;
            }
            break;

        case BSP_TOF_STATE_READY_POLL_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                if(0U == (tof_read_buffer[0] & 0x01U))
                {
                    tof_ready = 1U;
                    tof_state = BSP_TOF_STATE_READY;
                }
                else if(tof_ready_poll_count++ >= BSP_TOF_READY_MAX_POLLS)
                {
                    bsp_tof_fail(IIC_ERROR_TIMEOUT);
                }
                else
                {
                    tof_deadline_tick = now + BSP_TOF_READY_POLL_TICK;
                    tof_state = BSP_TOF_STATE_READY_POLL_DELAY;
                }
            }
            break;

        case BSP_TOF_STATE_READY:
            if(0U != tof_sample_requested)
            {
                tof_state = BSP_TOF_STATE_SAMPLE_GPIO_SUBMIT;
            }
            break;

        case BSP_TOF_STATE_SAMPLE_GPIO_SUBMIT:
            submit_result = bsp_tof_submit(DL1B_GPIO__TIO_HV_STATUS, 0U, 1U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_SAMPLE_GPIO_WAIT;
            }
            break;

        case BSP_TOF_STATE_SAMPLE_GPIO_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                if(0U == tof_read_buffer[0])
                {
                    tof_sample_requested = 0U;
                    tof_state = BSP_TOF_STATE_READY;
                }
                else
                {
                    tof_state = BSP_TOF_STATE_SAMPLE_CLEAR_SUBMIT;
                }
            }
            break;

        case BSP_TOF_STATE_SAMPLE_CLEAR_SUBMIT:
            tof_transfer_buffer[2] = 0x01U;
            submit_result = bsp_tof_submit(DL1B_SYSTEM__INTERRUPT_CLEAR, 1U, 0U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_SAMPLE_CLEAR_WAIT;
            }
            break;

        case BSP_TOF_STATE_SAMPLE_CLEAR_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                tof_state = BSP_TOF_STATE_SAMPLE_STATUS_SUBMIT;
            }
            break;

        case BSP_TOF_STATE_SAMPLE_STATUS_SUBMIT:
            submit_result = bsp_tof_submit(DL1B_RESULT__RANGE_STATUS, 0U, 1U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_SAMPLE_STATUS_WAIT;
            }
            break;

        case BSP_TOF_STATE_SAMPLE_STATUS_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                if(0x89U != tof_read_buffer[0])
                {
                    tof_distance_mm = BSP_TOF_INVALID_DISTANCE_MM;
                    tof_sample_requested = 0U;
                    tof_state = BSP_TOF_STATE_READY;
                }
                else
                {
                    tof_state = BSP_TOF_STATE_SAMPLE_DISTANCE_SUBMIT;
                }
            }
            break;

        case BSP_TOF_STATE_SAMPLE_DISTANCE_SUBMIT:
            submit_result = bsp_tof_submit(DL1B_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 0U, 2U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_STATE_SAMPLE_DISTANCE_WAIT;
            }
            break;

        case BSP_TOF_STATE_SAMPLE_DISTANCE_WAIT:
            if(0U != bsp_tof_transfer_succeeded())
            {
                tof_distance_mm = (uint16)(((uint16)tof_read_buffer[0] << 8) | tof_read_buffer[1]);
                if(tof_distance_mm > 4000U)
                {
                    tof_distance_mm = BSP_TOF_INVALID_DISTANCE_MM;
                }
                tof_sample_requested = 0U;
                tof_state = BSP_TOF_STATE_READY;
            }
            break;

        case BSP_TOF_STATE_ERROR:
            if(0U != bsp_tof_retry_deadline_reached(now))
            {
                (void)bsp_tof_async_init();
            }
            break;

        case BSP_TOF_STATE_IDLE:
        default:
            break;
    }
}

uint8 bsp_tof_async_is_ready(void)
{
    return tof_ready;
}

uint8 bsp_tof_async_request_sample(void)
{
    if(0U == tof_ready)
    {
        return 0U;
    }

    tof_sample_requested = 1U;
    return 1U;
}

uint16 bsp_tof_async_get_distance_mm(void)
{
    return (0U != tof_ready) ? tof_distance_mm : BSP_TOF_INVALID_DISTANCE_MM;
}

uint8 bsp_tof_async_get_range_status(void)
{
    return BSP_TOF_RANGE_STATUS_NO_UPDATE;
}

uint8 bsp_tof_async_get_last_error(void)
{
    return tof_last_error;
}

/* ------------------------------------------------------------------ */
/*  DL1A 零等待异步 IIC 状态机                                         */
/*  说明: 初始化 (dl1a_init) 仍为阻塞调用 (仅上电一次, 且发生在 wdt     */
/*        开启之前); 采样/读取路径全部基于 iic_async_transfer 提交,     */
/*        由主循环的 iic_async_process_all_timed 泵动, 调用点不阻塞。   */
/* ------------------------------------------------------------------ */
#elif (BSP_TOF_DRIVER == BSP_TOF_DRIVER_DL1A)

#define BSP_TOF_DL1A_START_WAIT_TICK           (100UL)      /* 10ms: 触发后最小等待 */
#define BSP_TOF_DL1A_POLL_DELAY_TICK           (50UL)       /* 5ms:  就绪轮询间隔   */
#define BSP_TOF_DL1A_STATUS_MAX_POLLS          (200U)       /* 1s 内未就绪则报错   */
#define BSP_TOF_DL1A_RETRY_WAIT_TICK           (10000UL)    /* 1s:  错误后重试      */
#define BSP_TOF_DL1A_SUBMIT_BUSY_TIMEOUT_TICK  (100UL)      /* 10ms: 总线忙重试上限 */

typedef enum
{
    BSP_TOF_DL1A_STATE_IDLE = 0,
    BSP_TOF_DL1A_STATE_START_SUBMIT,
    BSP_TOF_DL1A_STATE_START_WAIT,
    BSP_TOF_DL1A_STATE_STATUS_SUBMIT,
    BSP_TOF_DL1A_STATE_STATUS_WAIT,
    BSP_TOF_DL1A_STATE_POLL_DELAY,
    BSP_TOF_DL1A_STATE_RANGE_SUBMIT,
    BSP_TOF_DL1A_STATE_RANGE_WAIT,
    BSP_TOF_DL1A_STATE_CLEAR_SUBMIT,
    BSP_TOF_DL1A_STATE_CLEAR_WAIT,
    BSP_TOF_DL1A_STATE_ERROR,
} bsp_tof_dl1a_state_enum;

static uint8 tof_ready = 0U;
static uint8 tof_sample_requested = 0U;
static uint8 tof_last_error = 0U;
static uint8 xdata tof_transfer_buffer[2];
static uint8 xdata tof_read_buffer[12];
static bsp_tof_dl1a_state_enum tof_state = BSP_TOF_DL1A_STATE_IDLE;
static uint32 tof_deadline_tick = 0UL;
static uint32 tof_submit_busy_deadline_tick = 0UL;
static uint32 tof_retry_deadline_tick = 0UL;
static uint8 tof_submit_busy_pending = 0U;
static uint16 tof_status_poll_count = 0U;

static uint8 bsp_tof_dl1a_deadline_reached(uint32 now)
{
    return ((uint32)(now - tof_deadline_tick) < 0x80000000UL) ? 1U : 0U;
}

static uint8 bsp_tof_dl1a_parse_range_status(uint8 raw_status)
{
    uint8 device_range_status;

    /* RESULT_RANGE_STATUS[6:3] 为 VL53L0X 设备状态字段, 与 dl1a.c 的解析一致 */
    device_range_status = (uint8)((raw_status & 0x78U) >> 3);
    if((0U == device_range_status) ||
            (5U == device_range_status) ||
            (7U == device_range_status) ||
            (device_range_status >= 12U))
    {
        return DL1A_RANGE_STATUS_NO_UPDATE;
    }
    if(device_range_status <= 3U)
    {
        return 5U;       /* Hardware fail */
    }
    if((6U == device_range_status) || (9U == device_range_status))
    {
        return 4U;       /* Phase fail */
    }
    if((8U == device_range_status) || (10U == device_range_status))
    {
        return 3U;       /* Min range fail */
    }
    if(4U == device_range_status)
    {
        return 2U;       /* Signal fail */
    }
    return 0U;           /* Range valid (device status 11) */
}

static void bsp_tof_dl1a_fail(iic_status_enum status)
{
    tof_last_error = (uint8)status;
    tof_submit_busy_pending = 0U;
    tof_retry_deadline_tick = service_timetick_what() + BSP_TOF_DL1A_RETRY_WAIT_TICK;
    tof_state = BSP_TOF_DL1A_STATE_ERROR;
}

static uint8 bsp_tof_dl1a_submit(uint8 reg, uint8 data_byte, uint8 *read_data, uint8 read_len)
{
    iic_status_enum status;
    uint32 now;

    tof_transfer_buffer[0] = reg;
    if(0U == read_len)
    {
        tof_transfer_buffer[1] = data_byte;
    }
    status = iic_async_transfer(DL1A_IIC, DL1A_DEV_ADDR,
            tof_transfer_buffer, (uint32)(1U + ((0U == read_len) ? 1U : 0U)), read_data, read_len);
    if(IIC_SUCCESS == status)
    {
        tof_submit_busy_pending = 0U;
        return 1U;
    }
    if(IIC_ERROR_BUSY == status)
    {
        now = service_timetick_what();
        if(0U == tof_submit_busy_pending)
        {
            tof_submit_busy_pending = 1U;
            tof_submit_busy_deadline_tick = now + BSP_TOF_DL1A_SUBMIT_BUSY_TIMEOUT_TICK;
            return 0U;
        }
        if((uint32)(now - tof_submit_busy_deadline_tick) < 0x80000000UL)
        {
            bsp_tof_dl1a_fail(IIC_ERROR_TIMEOUT);
            return 2U;
        }
        return 0U;
    }
    bsp_tof_dl1a_fail(status);
    return 2U;
}

static uint8 bsp_tof_dl1a_transfer_succeeded(void)
{
    iic_status_enum status;

    if(0U != iic_async_is_busy(DL1A_IIC))
    {
        return 0U;
    }
    status = iic_async_get_status(DL1A_IIC);
    if(IIC_SUCCESS != status)
    {
        bsp_tof_dl1a_fail(status);
        return 0U;
    }
    return 1U;
}

uint8 bsp_tof_async_init(void)
{
    tof_last_error = dl1a_init();
    tof_ready = (0U == tof_last_error) ? 1U : 0U;
    tof_sample_requested = 0U;
    tof_submit_busy_pending = 0U;
    tof_status_poll_count = 0U;
    tof_deadline_tick = 0UL;
    tof_retry_deadline_tick = 0UL;
    tof_state = BSP_TOF_DL1A_STATE_IDLE;
    return tof_ready;
}

void bsp_tof_async_process(void)
{
    uint8 submit_result;
    uint32 now;

    if(0U == tof_ready)
    {
        return;
    }
    now = service_timetick_what();
    switch(tof_state)
    {
        case BSP_TOF_DL1A_STATE_IDLE:
            if(0U != tof_sample_requested)
            {
                tof_state = BSP_TOF_DL1A_STATE_START_SUBMIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_START_SUBMIT:
            submit_result = bsp_tof_dl1a_submit(DL1A_SYSRANGE_START, 0x02U, NULL, 0U);
            if(1U == submit_result)
            {
                tof_sample_requested = 0U;
                tof_status_poll_count = 0U;
                tof_deadline_tick = now + BSP_TOF_DL1A_START_WAIT_TICK;
                tof_state = BSP_TOF_DL1A_STATE_START_WAIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_START_WAIT:
            if(0U != bsp_tof_dl1a_transfer_succeeded())
            {
                tof_deadline_tick = now + BSP_TOF_DL1A_START_WAIT_TICK;
                tof_state = BSP_TOF_DL1A_STATE_STATUS_SUBMIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_STATUS_SUBMIT:
            submit_result = bsp_tof_dl1a_submit(DL1A_RESULT_INTERRUPT_STATUS, 0x00U, tof_read_buffer, 1U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_DL1A_STATE_STATUS_WAIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_STATUS_WAIT:
            if(0U != bsp_tof_dl1a_transfer_succeeded())
            {
                if(0U != (tof_read_buffer[0] & 0x07U))
                {
                    tof_state = BSP_TOF_DL1A_STATE_RANGE_SUBMIT;
                }
                else if(tof_status_poll_count++ >= BSP_TOF_DL1A_STATUS_MAX_POLLS)
                {
                    bsp_tof_dl1a_fail(IIC_ERROR_TIMEOUT);
                }
                else
                {
                    tof_deadline_tick = now + BSP_TOF_DL1A_POLL_DELAY_TICK;
                    tof_state = BSP_TOF_DL1A_STATE_POLL_DELAY;
                }
            }
            break;

        case BSP_TOF_DL1A_STATE_POLL_DELAY:
            if(0U != bsp_tof_dl1a_deadline_reached(now))
            {
                tof_state = BSP_TOF_DL1A_STATE_STATUS_SUBMIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_RANGE_SUBMIT:
            submit_result = bsp_tof_dl1a_submit(DL1A_RESULT_RANGE_STATUS, 0x00U, tof_read_buffer, 12U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_DL1A_STATE_RANGE_WAIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_RANGE_WAIT:
            if(0U != bsp_tof_dl1a_transfer_succeeded())
            {
                dl1a_range_status = bsp_tof_dl1a_parse_range_status(tof_read_buffer[0]);
                dl1a_distance_mm = (uint16)(((uint16)tof_read_buffer[10] << 8) | tof_read_buffer[11]);
                if(dl1a_distance_mm > 8191U)
                {
                    dl1a_distance_mm = 8192U;
                    dl1a_range_status = DL1A_RANGE_STATUS_NO_UPDATE;
                }
                tof_state = BSP_TOF_DL1A_STATE_CLEAR_SUBMIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_CLEAR_SUBMIT:
            submit_result = bsp_tof_dl1a_submit(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01U, NULL, 0U);
            if(1U == submit_result)
            {
                tof_state = BSP_TOF_DL1A_STATE_CLEAR_WAIT;
            }
            break;

        case BSP_TOF_DL1A_STATE_CLEAR_WAIT:
            if(0U != bsp_tof_dl1a_transfer_succeeded())
            {
                tof_state = BSP_TOF_DL1A_STATE_IDLE;
            }
            break;

        case BSP_TOF_DL1A_STATE_ERROR:
            if((uint32)(now - tof_retry_deadline_tick) < 0x80000000UL)
            {
                tof_state = BSP_TOF_DL1A_STATE_IDLE;
            }
            break;

        default:
            break;
    }
}

uint8 bsp_tof_async_is_ready(void)
{
    return tof_ready;
}

uint8 bsp_tof_async_request_sample(void)
{
    if(0U == tof_ready)
    {
        return 0U;
    }
    tof_sample_requested = 1U;
    return 1U;
}

uint16 bsp_tof_async_get_distance_mm(void)
{
    return (0U != tof_ready) ? dl1a_distance_mm : BSP_TOF_INVALID_DISTANCE_MM;
}

uint8 bsp_tof_async_get_range_status(void)
{
    return (0U != tof_ready) ? dl1a_range_status : BSP_TOF_RANGE_STATUS_NO_UPDATE;
}

uint8 bsp_tof_async_get_last_error(void)
{
    return tof_last_error;
}

#endif
