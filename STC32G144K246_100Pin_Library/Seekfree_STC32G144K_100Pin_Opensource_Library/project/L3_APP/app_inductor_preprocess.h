#ifndef APP_INDUCTOR_PREPROCESS_H
#define APP_INDUCTOR_PREPROCESS_H

#include "zf_common_typedef.h"

#ifndef APP_INDUCTOR_PREPROCESS_PIT
#define APP_INDUCTOR_PREPROCESS_PIT            TIM4_PIT
#endif

#ifndef APP_INDUCTOR_PREPROCESS_PERIOD_US
#define APP_INDUCTOR_PREPROCESS_PERIOD_US      (1000U)
#endif

typedef struct
{
    float filtered[4];
    float normalized[4];
} app_inductor_preprocess_data_t;

extern uint16 app_inductor_preprocess_min_value[4];
extern uint16 app_inductor_preprocess_max_value[4];

void app_inductor_preprocess_init(void);
void app_inductor_preprocess_debug(void);
void app_inductor_preprocess_update_calibration(void);
void app_inductor_preprocess_get_data(app_inductor_preprocess_data_t *out_data);

#endif
