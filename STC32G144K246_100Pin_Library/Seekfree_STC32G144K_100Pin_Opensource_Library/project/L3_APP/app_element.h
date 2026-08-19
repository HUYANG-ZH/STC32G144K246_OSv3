#ifndef APP_ELEMENT_H
#define APP_ELEMENT_H

#include "zf_common_typedef.h"
#include "service_imu.h"

#ifndef APP_ELEMENT_CYLINDER_YAW_LIMIT_POS
#define APP_ELEMENT_CYLINDER_YAW_LIMIT_POS      (5.0f)      // 圆筒元素触发后车体转向角速度正方向限幅，单位 rad/s
#endif

#ifndef APP_ELEMENT_CYLINDER_YAW_LIMIT_NEG
#define APP_ELEMENT_CYLINDER_YAW_LIMIT_NEG      (5.0f)      // 圆筒元素触发后车体转向角速度负方向限幅，单位 rad/s
#endif

#ifndef APP_ELEMENT_ROUNDABOUT_BIAS_BLEND
#define APP_ELEMENT_ROUNDABOUT_BIAS_BLEND       (0.72f)      // 环岛偏置融合系数，0=仅巡线 1=仅偏置
#endif

#ifndef APP_ELEMENT_PIT
#define APP_ELEMENT_PIT                         TIM7_PIT
#endif

#ifndef APP_ELEMENT_PERIOD_MS
#define APP_ELEMENT_PERIOD_MS                   (5U)
#endif

#ifndef APP_ELEMENT_ROUNDABOUT_PIT
#define APP_ELEMENT_ROUNDABOUT_PIT              TIM7_PIT
#endif

typedef enum
{
    APP_ELEMENT_TYPE_NONE = 0,
    APP_ELEMENT_TYPE_ROUNDABOUT,
    APP_ELEMENT_TYPE_CROSSROAD,
    APP_ELEMENT_TYPE_LOST_LINE,
    APP_ELEMENT_TYPE_CYLINDER,
    APP_ELEMENT_TYPE_SEESAW,
    APP_ELEMENT_TYPE_UPHILL,
} app_element_type_t;

typedef enum
{
    APP_ELEMENT_STATE_IDLE = 0,
    APP_ELEMENT_STATE_ENTER,
    APP_ELEMENT_STATE_INSIDE,
    APP_ELEMENT_STATE_EXIT,
    APP_ELEMENT_STATE_DONE,
} app_element_state_t;

typedef enum
{
    APP_ELEMENT_DIR_NONE = 0,
    APP_ELEMENT_DIR_LEFT,
    APP_ELEMENT_DIR_RIGHT,
} app_element_dir_t;

typedef struct
{
    float roundabout_enable;
    float roundabout_enter_error;
    float roundabout_exit_error;
    float roundabout_signal_min;
    uint16 roundabout_enter_ms;
    uint16 roundabout_inside_ms;
    uint16 roundabout_exit_ms;
    uint16 roundabout_done_ms;
    float crossroad_enable;
    float crossroad_center_error;
    float crossroad_signal_min;
    float crossroad_signal_sum_min;
    uint16 crossroad_enter_ms;
    uint16 crossroad_inside_ms;
    uint16 crossroad_exit_ms;
    uint16 crossroad_done_ms;
    float lost_line_signal_sum_max;
    uint16 lost_line_confirm_ms;
} app_element_config_t;

typedef struct
{
    app_element_type_t type;
    app_element_state_t state;
    app_element_dir_t dir;
    float active;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} app_element_data_t;

extern app_element_config_t app_element_config;

extern volatile float app_element_roundabout_bias_yaw_radps;
extern volatile uint8 app_element_roundabout_bias_active;
extern volatile float app_element_roundabout_feedforward_scale;

void app_element_init(void);
void app_element_get_data(app_element_data_t *out_data);
/* 5 ms 控制链步骤；不再自行占用低优先级队列或独立定时器。 */
void app_element_control_step(void);
/* Receives the same unified IMU snapshot consumed by the control step. */
void app_element_imu_task(const service_imu_sample_t *imu);
void app_element_pump_events(void);
uint8 app_element_seesaw_is_slowdown_active(void);
float app_element_seesaw_slowdown_speed_mps(void);

#endif
