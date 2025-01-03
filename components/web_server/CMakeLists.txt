set(requires esp_http_server)
idf_build_get_property(target IDF_TARGET)
if(${target} STREQUAL "linux")
  list(APPEND requires esp_stubs)
endif()

idf_component_register(SRCS  INCLUDE_DIRS "include" REQUIRES
                       ${requires} PRIV_REQUIRES power  app lwip esp_app_format)
