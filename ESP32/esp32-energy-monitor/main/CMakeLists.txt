idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES manejo_wifi ntp_sync modbus_rs485 mqtt_client led_status)

