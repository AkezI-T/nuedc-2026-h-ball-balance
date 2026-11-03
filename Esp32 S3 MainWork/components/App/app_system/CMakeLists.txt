idf_component_register(
    SRCS "app_system.c"
    INCLUDE_DIRS "include"
    REQUIRES app_wifi app_encoder_monitor app_jy901s app_motor_control
             app_task_dispatch app_udp_log app_vision app_tmx_ingest
             bsp_fill_light)
