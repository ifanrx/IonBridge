idf_component_register(
    SRCS "config_sync_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_timer
        mqtt_app
        nvs_flash
        port
        power
        pb
        utils
        display
    PRIV_REQUIRES
        esp_system
        nvs_data
)
