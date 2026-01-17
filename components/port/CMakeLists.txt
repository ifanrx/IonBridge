file(GLOB SRCS_FILES *.cpp)
set(REQUIRE_COMPONENTS chip_data_types esp_timer utils)
set(PRIV_REQUIRE_COMPONENTS acdc display machine_info power rpc task)

include(${PROJECT_DIR}/component_requires.cmake)
if(ENABLE_COMPONENT_UART)
  list(APPEND REQUIRE_COMPONENTS uart)
  if(ENABLE_COMPONENT_PROGRAMMER)
    list(APPEND PRIV_REQUIRE_COMPONENTS programmer)
  endif()
endif()
if(ENABLE_COMPONENT_I2C)
  list(APPEND REQUIRE_COMPONENTS i2c)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  ${REQUIRE_COMPONENTS}
  PRIV_REQUIRES
  ${PRIV_REQUIRE_COMPONENTS})
