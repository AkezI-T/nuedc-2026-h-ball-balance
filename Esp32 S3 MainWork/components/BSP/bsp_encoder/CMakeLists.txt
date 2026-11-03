idf_component_register(
    SRCS "bsp_encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_mcpwm esp_driver_pcnt)
