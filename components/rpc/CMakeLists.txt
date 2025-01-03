set(SRCS_FILES rpc.cpp mcu.cpp)

if(CONFIG_MCU_MODEL_SW3566)
  list(APPEND SRCS_FILES fpga.cpp sw3566.cpp)
elseif(CONFIG_MCU_MODEL_FAKE_SW3566)
  list(APPEND SRCS_FILES fakefpga.cpp fakechip.cpp)
elseif(CONFIG_MCU_MODEL_SW3526_SW3536)
  list(APPEND SRCS_FILES sw3526.cpp display.cpp)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  chip_data_types
  PRIV_REQUIRES
  animation
  driver
  storage
  utils)

# https://github.com/espressif/esp-idf/issues/10810
if(ENABLE_COMPONENT_UART)
  target_link_libraries(${COMPONENT_LIB} PRIVATE idf::uart)
elseif(ENABLE_COMPONENT_I2C)
  target_link_libraries(${COMPONENT_LIB} PRIVATE idf::i2c)
endif()
