idf_component_register(
    SRCS "app_encoder_monitor.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common app_tmx_ingest app_udp_log bsp_encoder)
