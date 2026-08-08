#ifndef APP_INDUCTOR_PREPROCESS_H
#define APP_INDUCTOR_PREPROCESS_H

#include "zf_common_typedef.h"

#ifndef APP_INDUCTOR_PREPROCESS_PIT
#define APP_INDUCTOR_PREPROCESS_PIT            TIM4_PIT
#endif

#ifndef APP_INDUCTOR_PREPROCESS_PERIOD_US
#define APP_INDUCTOR_PREPROCESS_PERIOD_US      (1000U)
#endif

#define APP_INDUCTOR_PREPROCESS_SIDE_COUNT     (4U)
#define APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT  (5U)

#define APP_INDUCTOR_PREPROCESS_INDEX_CH1      (0U)
#define APP_INDUCTOR_PREPROCESS_INDEX_CH2      (1U)
#define APP_INDUCTOR_PREPROCESS_INDEX_CH3      (2U)
#define APP_INDUCTOR_PREPROCESS_INDEX_CH4      (3U)
#define APP_INDUCTOR_PREPROCESS_INDEX_M        (4U)

typedef struct
{
    float filtered[APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT];
    float normalized[APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT];
} app_inductor_preprocess_data_t;

extern uint16 app_inductor_preprocess_min_value[APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT];
extern uint16 app_inductor_preprocess_max_value[APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT];

void app_inductor_preprocess_init(void);
void app_inductor_preprocess_debug(void);
void app_inductor_preprocess_get_data(app_inductor_preprocess_data_t *out_data);

#endif
