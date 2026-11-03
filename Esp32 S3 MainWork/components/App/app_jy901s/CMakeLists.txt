idf_component_register(
    SRCS "app_jy901s.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common esp_driver_i2c esp_timer)
