message(STATUS "Including Component RPC")
set(SRCS_FILES rpc.cpp mcu.cpp)
set(PRIV_REQUIRE_COMPONENTS display driver esp_timer storage utils)

include(${PROJECT_DIR}/component_requires.cmake)

if(CONFIG_MCU_MODEL_SW3566)
  if(ENABLE_COMPONENT_UART)
    list(APPEND SRCS_FILES fpga.cpp sw3566.cpp)
  else()
    message(WARNING "SW3566 requires UART but UART is disabled. FPGA/SW3566 support will not be compiled.")
  endif()
elseif(CONFIG_MCU_MODEL_FAKE_SW3566)
  list(APPEND SRCS_FILES fakefpga.cpp fakechip.cpp)
elseif(CONFIG_MCU_MODEL_SW3536)
  list(APPEND SRCS_FILES sw3536.cpp)
elseif(CONFIG_MCU_MODEL_SW3526)
  list(APPEND SRCS_FILES sw3526.cpp)
elseif(CONFIG_MCU_MODEL_SW3516)
  list(APPEND SRCS_FILES sw3516.cpp)
endif()

if(CONFIG_DISPLAY_DRIVER_GPU)
  message(STATUS "RPC: Adding GPU display driver support")
  if(ENABLE_COMPONENT_UART)
    list(APPEND SRCS_FILES gpu.cpp)
  else()
    message(WARNING "GPU display driver requires UART but UART is disabled. GPU support will not be compiled.")
  endif()
endif()

if(ENABLE_COMPONENT_UART)
  message(STATUS "RPC: Adding UART interface support")
  list(APPEND PRIV_REQUIRE_COMPONENTS uart)
endif()
if(ENABLE_COMPONENT_I2C)
  message(STATUS "RPC: Adding I2C interface support")
  list(APPEND PRIV_REQUIRE_COMPONENTS i2c)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  chip_data_types
  version
  PRIV_REQUIRES
  ${PRIV_REQUIRE_COMPONENTS})
