file(GLOB SRCS_FILES *.cpp)
set(TASK_PRIV_REQUIRE_COMPONENTS
    acdc
    app
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
    utils
    wifi)

idf_build_get_property(target IDF_TARGET)
if(NOT ${target} STREQUAL "linux")
  list(APPEND TASK_PRIV_REQUIRE_COMPONENTS espcoredump)
endif()

include(${PROJECT_DIR}/component_requires.cmake)
if(ENABLE_COMPONENT_UART)
  list(APPEND TASK_PRIV_REQUIRE_COMPONENTS uart)
  if(ENABLE_COMPONENT_PROGRAMMER)
    list(APPEND TASK_PRIV_REQUIRE_COMPONENTS programmer)
  endif()
endif()
if(ENABLE_COMPONENT_I2C)
  list(APPEND TASK_PRIV_REQUIRE_COMPONENTS i2c)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  chip_data_types
  pb
  service
  version
  PRIV_REQUIRES
  ${TASK_PRIV_REQUIRE_COMPONENTS})
