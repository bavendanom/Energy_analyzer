#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t modbus_rs485_init(void);
esp_err_t modbus_read_parameters(void);

#ifdef __cplusplus
}
#endif
