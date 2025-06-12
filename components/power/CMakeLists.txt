file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  port
  PRIV_REQUIRES
  chip_data_types
  esp_timer
  machine_info
  nvs_data
  utils)
set_property(
  TARGET ${COMPONENT_LIB}
  APPEND
  PROPERTY LINK_INTERFACE_MULTIPLICITY 3)
