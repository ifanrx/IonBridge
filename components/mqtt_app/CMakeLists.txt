file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  mqtt
  power
  rpc
  utils
  PRIV_REQUIRES
  esp_wifi
  app
  ble
  nvs_data
  task
  wifi)
