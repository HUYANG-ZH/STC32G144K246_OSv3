#ifndef APP_LOG_H
#define APP_LOG_H

#include "zf_common_typedef.h"

void app_log_init(void);
uint32 Wlog(const char *format, ...);
void Rlog(void);
void Clog(void);

#endif
