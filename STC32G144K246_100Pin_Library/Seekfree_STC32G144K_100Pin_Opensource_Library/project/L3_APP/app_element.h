#ifndef APP_ELEMENT_H
#define APP_ELEMENT_H

#include "zf_common_typedef.h"

#ifndef APP_ELEMENT_PIT
#define APP_ELEMENT_PIT                         TIM7_PIT
#endif

#ifndef APP_ELEMENT_PERIOD_MS
#define APP_ELEMENT_PERIOD_MS                   (5U)
#endif

typedef enum
{
    APP_ELEMENT_TYPE_NONE = 0,
    APP_ELEMENT_TYPE_ROUNDABOUT,
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
} app_element_config_t;

typedef struct
{
    app_element_type_t type;
    app_element_state_t state;
    app_element_dir_t dir;
    float active;
} app_element_data_t;

extern app_element_config_t app_element_config;

void app_element_init(void);
void app_element_get_data(app_element_data_t *out_data);

#endif
