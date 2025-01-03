file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  port
  rpc
  uart
  PRIV_REQUIRES
  app
  controller
  mqtt_app
  nvs_data
  task
  utils)
