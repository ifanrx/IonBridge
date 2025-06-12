include(${PROJECT_DIR}/component_requires.cmake)
if(ENABLE_COMPONENT_UART)
  message(STATUS "Including Component UART")
  file(GLOB SRCS_FILES *.cpp)
  idf_component_register(
    SRCS
    ${SRCS_FILES}
    INCLUDE_DIRS
    "include"
    PRIV_REQUIRES
    esp_driver_uart
    esp_timer
    port
    utils)
else()
  message(STATUS "Skipping Component UART")
endif()
