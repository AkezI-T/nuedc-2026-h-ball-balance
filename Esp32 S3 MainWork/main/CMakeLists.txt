idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        app_common
        app_jy901s
        app_system
        app_motor_control
        app_task_dispatch
        app_tmx_ingest
        app_udp_log
        app_vision
        esp_timer)
