file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  controller
  power
  service
  PRIV_REQUIRES
  mqtt_app
  nvs_data
  version)
