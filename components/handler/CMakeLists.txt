file(GLOB SRCS_FILES *.cpp)
set(PRIV_REQUIRED_COMPONENTS
    acdc
    app
    config_sync
    controller
    device_data
    display
    ble
    efuse
    esp_http_client
    esp_wifi
    firmware
    fpga
    license
    logging
    machine_info
    mqtt_app
    nvs_data
    pb
    port
    power
    rpc
    task
    utils
    wifi)

include(${PROJECT_DIR}/component_requires.cmake)
if(ENABLE_COMPONENT_UART AND ENABLE_COMPONENT_PROGRAMMER)
  list(APPEND PRIV_REQUIRED_COMPONENTS programmer)
endif()

idf_component_register(SRCS ${SRCS_FILES} INCLUDE_DIRS "include" PRIV_REQUIRES
                       ${PRIV_REQUIRED_COMPONENTS})
