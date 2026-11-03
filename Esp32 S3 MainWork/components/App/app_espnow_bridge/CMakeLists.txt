idf_component_register(
    SRCS "app_espnow_bridge.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common esp_wifi app_wifi)
