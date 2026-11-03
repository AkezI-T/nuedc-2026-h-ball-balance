idf_component_register(
    SRCS "app_tmx_ingest.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common bsp_uart app_udp_log)
