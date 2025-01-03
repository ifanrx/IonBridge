file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  rpc
  PRIV_REQUIRES
  app
  utils)

if(ENABLE_COMPONENT_UART)
  target_link_libraries(${COMPONENT_LIB} PUBLIC idf::uart)
elseif(ENABLE_COMPONENT_I2C)
  target_link_libraries(${COMPONENT_LIB} PUBLIC idf::i2c)
endif()
