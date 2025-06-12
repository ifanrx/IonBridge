file(GLOB SRCS_FILES *.cpp)

set(PRIV_REQUIRED_COMPONENTS nvs_data utils)
set(REQUIRED_COMPONENTS)

include(${PROJECT_DIR}/component_requires.cmake)
if(ENABLE_COMPONENT_I2C)
  list(APPEND REQUIRED_COMPONENTS lvgl)
  list(APPEND PRIV_REQUIRED_COMPONENTS esp_lcd esp_lvgl_port bsp_generic port)
else()
  list(APPEND PRIV_REQUIRED_COMPONENTS rpc)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  ${REQUIRED_COMPONENTS}
  PRIV_REQUIRES
  ${PRIV_REQUIRED_COMPONENTS})
