#include "zf_common_headfile.h"
#include "service_imu.h"
#include "app_element.h"

app_element_config_t app_element_config =
{
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    0U,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    0U,
    0.0f,
    0U
};

static volatile app_element_data_t element_data =
{
    APP_ELEMENT_TYPE_NONE,
    APP_ELEMENT_STATE_IDLE,
    APP_ELEMENT_DIR_NONE,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

static void app_element_reset(void)
{
    element_data.type = APP_ELEMENT_TYPE_NONE;
    element_data.state = APP_ELEMENT_STATE_IDLE;
    element_data.dir = APP_ELEMENT_DIR_NONE;
    element_data.active = 0.0f;
    element_data.gyro_x = 0.0f;
    element_data.gyro_y = 0.0f;
    element_data.gyro_z = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     元素识别初始化
// 参数说明     void
// 返回参数     void
// 使用示例     app_element_init();
// 备注信息     当前元素处理清空重做，初始化后默认无元素
//-------------------------------------------------------------------------------------------------------------------
void app_element_init(void)
{
    uint8 ea_backup;

    ea_backup = EA;
    EA = 0;
    app_element_reset();
    EA = ea_backup;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取元素识别数据
// 参数说明     out_data        数据输出地址
// 返回参数     void
// 使用示例     app_element_get_data(&element_data);
//-------------------------------------------------------------------------------------------------------------------
void app_element_get_data(app_element_data_t *out_data)
{
    uint8 ea_backup;

    if(NULL == out_data)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    *out_data = element_data;
    EA = ea_backup;
}

void app_element_imu_task(const service_imu_gyro_t *gyro)
{
    uint8 ea_backup;

    if(NULL == gyro)
    {
        return;
    }

    ea_backup = EA;
    EA = 0;
    element_data.gyro_x = gyro->gyro_x;
    element_data.gyro_y = gyro->gyro_y;
    element_data.gyro_z = gyro->gyro_z;
    EA = ea_backup;
}
