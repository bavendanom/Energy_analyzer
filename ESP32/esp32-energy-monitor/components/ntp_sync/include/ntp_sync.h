#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ntp_sync_time(const char *ntp_server);

#ifdef __cplusplus
}
#endif
