include(${PROJECT_DIR}/component_requires.cmake)
set(PRIV_REQUIRES esp_driver_gpio port)
if(ENABLE_COMPONENT_UART)
  list(APPEND PRIV_REQUIRES chip_data_types)
endif()
file(GLOB SRCS_FILES *.cpp)
idf_component_register(SRCS ${SRCS_FILES} INCLUDE_DIRS "include" PRIV_REQUIRES
                       ${PRIV_REQUIRES})
