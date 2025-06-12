set(requires esp_http_server json)
idf_build_get_property(target IDF_TARGET)
if(${target} STREQUAL "linux")
  list(APPEND requires esp_stubs)
endif()

idf_component_register(
  SRCS
  "server.cpp"
  INCLUDE_DIRS
  "include"
  REQUIRES
  ${requires}
  PRIV_REQUIRES
  display
  esp_app_format
  esp_partition
  esp_wifi
  espcoredump
  logging
  machine_info
  mqtt_app
  nvs_data
  power
  wifi)
