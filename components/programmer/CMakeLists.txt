include(${PROJECT_DIR}/component_requires.cmake)

if(NOT ENABLE_COMPONENT_PROGRAMMER)
  message(STATUS "Programmer component disabled")
  return()
endif()

if(NOT ENABLE_COMPONENT_UART)
  message(STATUS "Programmer component disabled - UART interface required")
  return()
endif()

message(STATUS "Including Component PROGRAMMER")
set(REQUIRE_COMPONENTS
    freertos
    version
    task
    storage
    firmware
    rpc
    mqtt_app
    uart
    display
    controller
)

idf_component_register(SRCS "firmware_programmer.cpp" INCLUDE_DIRS "include"
                       REQUIRES ${REQUIRE_COMPONENTS})
