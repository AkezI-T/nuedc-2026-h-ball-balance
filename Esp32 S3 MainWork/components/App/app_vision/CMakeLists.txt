idf_component_register(
    SRCS "app_vision.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common bsp_uart esp_timer)
