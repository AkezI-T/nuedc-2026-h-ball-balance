idf_component_register(
    SRCS "app_task_dispatch.c"
    INCLUDE_DIRS "include"
    REQUIRES app_tmx_ingest app_udp_log app_vision)
