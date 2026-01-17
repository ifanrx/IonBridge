idf_component_register(SRCS "device_data.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_common
                    PRIV_REQUIRES nvs_data pb)
