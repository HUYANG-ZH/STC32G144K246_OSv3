#ifndef SERVICE_TOF_H
#define SERVICE_TOF_H

#include "zf_common_typedef.h"

/*
 * Board configuration: the current vehicle has no ToF fitted.
 * Keep the service API linked so that enabling the sensor later only requires
 * changing this switch to 1; while disabled no IIC1/P76/P77 or XSHUT access
 * is made by the ToF service.
 */
#ifndef SERVICE_TOF_ENABLE
#define SERVICE_TOF_ENABLE             (1U)
#endif

void service_tof_init(void);
void service_tof_task(void);
uint8 service_tof_is_ready(void);
uint8 service_tof_get_last_error(void);
uint16 service_tof_get_distance_mm(void);

#endif
