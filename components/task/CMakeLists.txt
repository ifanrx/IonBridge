file(GLOB SRCS_FILES *.cpp)
set(PRIV_REQUIRE_COMPONENTS
    app_update
    ble
    controller
    display
    esp_timer
    esp_wifi
    littlefs
    machine_info
    mqtt_app
    nvs_data
    power
    rpc
    service
    wifi)

include(${PROJECT_DIR}/component_requires.cmake)
if(ENABLE_COMPONENT_UART)
  list(APPEND PRIV_REQUIRE_COMPONENTS uart)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  mqtt_message
  version
  PRIV_REQUIRES
  ${PRIV_REQUIRE_COMPONENTS})
