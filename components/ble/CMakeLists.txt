file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  esp_nimble_cpp
  PRIV_REQUIRES
  app
  display
  esp_timer
  machine_info
  nvs_data
  service
  utils
  wifi)
