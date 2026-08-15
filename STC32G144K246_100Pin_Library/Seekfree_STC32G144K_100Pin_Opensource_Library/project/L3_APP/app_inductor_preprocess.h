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
/* car3 已彻底弃用 M 通道电感, 仅保留 CH1~CH4 四路 */
#define APP_INDUCTOR_PREPROCESS_CHANNEL_COUNT  (4U)

/* 归一化软上限: 允许近场线性外推超过 100, 仅兜底传感器故障/标定误调 */
#define APP_INDUCTOR_NORM_UPPER_LIMIT          (300.0f)

#define APP_INDUCTOR_PREPROCESS_INDEX_CH1      (0U)
#define APP_INDUCTOR_PREPROCESS_INDEX_CH2      (1U)
#define APP_INDUCTOR_PREPROCESS_INDEX_CH3      (2U)
#define APP_INDUCTOR_PREPROCESS_INDEX_CH4      (3U)

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
