#ifndef SERVICE_BUTTON_H
#define SERVICE_BUTTON_H

#include "zf_common_typedef.h"

void service_button_init(void);
void service_button_task(void);
uint8 service_button_clicked(void);
uint8 service_button_get_state(void);

#endif
