idf_component_register(
    SRCS "app_motor_control.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common app_jy901s app_task_dispatch app_tmx_ingest app_vision
             bsp_d36a bsp_encoder)
