#include "zf_common_headfile.h"
#include "bsp_encoder.h"
#include "zf_driver_encoder.h"

#ifndef BSP_ENCODER_LEFT_INDEX
#define BSP_ENCODER_LEFT_INDEX         PWMF_ENCODER
#endif

#ifndef BSP_ENCODER_LEFT_PULSE_PIN
#define BSP_ENCODER_LEFT_PULSE_PIN     PWMF_ENCODER_CH1_PA1
#endif

#ifndef BSP_ENCODER_LEFT_DIR_PIN
#define BSP_ENCODER_LEFT_DIR_PIN       PWMF_ENCODER_CH2_PA3
#endif

#ifndef BSP_ENCODER_LEFT_SIGN
#define BSP_ENCODER_LEFT_SIGN          (1)
#endif

#ifndef BSP_ENCODER_RIGHT_INDEX
#define BSP_ENCODER_RIGHT_INDEX        PWME_ENCODER
#endif

#ifndef BSP_ENCODER_RIGHT_PULSE_PIN
#define BSP_ENCODER_RIGHT_PULSE_PIN    PWME_ENCODER_CH1P_PA0
#endif

#ifndef BSP_ENCODER_RIGHT_DIR_PIN
#define BSP_ENCODER_RIGHT_DIR_PIN      PWME_ENCODER_CH2P_PA2
#endif

#ifndef BSP_ENCODER_RIGHT_SIGN
#define BSP_ENCODER_RIGHT_SIGN          (-1)
#endif

void bsp_encoder_init(void)
{
    encoder_dir_init(BSP_ENCODER_LEFT_INDEX, BSP_ENCODER_LEFT_PULSE_PIN, BSP_ENCODER_LEFT_DIR_PIN);
    encoder_dir_init(BSP_ENCODER_RIGHT_INDEX, BSP_ENCODER_RIGHT_PULSE_PIN, BSP_ENCODER_RIGHT_DIR_PIN);
    bsp_encoder_clear();
}

void bsp_encoder_debug(void)
{
}

void bsp_encoder_clear(void)
{
    encoder_clear_count(BSP_ENCODER_LEFT_INDEX);
    encoder_clear_count(BSP_ENCODER_RIGHT_INDEX);
}

void bsp_encoder_get_delta(bsp_encoder_count_t *out_count)
{
    if(NULL == out_count)
    {
        return;
    }

    out_count->left = encoder_get_count(BSP_ENCODER_LEFT_INDEX) * BSP_ENCODER_LEFT_SIGN;
    out_count->right = encoder_get_count(BSP_ENCODER_RIGHT_INDEX) * BSP_ENCODER_RIGHT_SIGN;

    bsp_encoder_clear();
}
