#ifndef SHARED_LPF_H
#define SHARED_LPF_H

typedef struct
{
    float alpha;
    float output;
} shared_lpf_t;

void shared_lpf_init(shared_lpf_t *filter, float alpha, float init_value);
void shared_lpf_set_alpha(shared_lpf_t *filter, float alpha);
void shared_lpf_reset(shared_lpf_t *filter, float value);
float shared_lpf_update(shared_lpf_t *filter, float input);
float shared_lpf_get(const shared_lpf_t *filter);

#endif
