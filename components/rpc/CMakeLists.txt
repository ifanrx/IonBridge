set(SRCS_FILES rpc.cpp mcu.cpp)
set(PRIV_REQUIRE_COMPONENTS display driver esp_timer storage utils)

if(CONFIG_MCU_MODEL_SW3566)
  list(APPEND SRCS_FILES fpga.cpp sw3566.cpp)
elseif(CONFIG_MCU_MODEL_FAKE_SW3566)
  list(APPEND SRCS_FILES fakefpga.cpp fakechip.cpp)
endif()

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
  chip_data_types
  PRIV_REQUIRES
  ${PRIV_REQUIRE_COMPONENTS})
