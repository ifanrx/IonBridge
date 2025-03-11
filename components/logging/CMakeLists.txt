file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  esp_ringbuf
  PRIV_REQUIRES
  esp_wifi
  app
  nvs_data
  wifi)
