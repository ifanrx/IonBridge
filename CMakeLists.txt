# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS components/)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

include(component_requires.cmake)
# Print the status of options
message(STATUS "ENABLE_COMPONENT_UART: ${ENABLE_COMPONENT_UART}")
message(STATUS "ENABLE_COMPONENT_I2C: ${ENABLE_COMPONENT_I2C}")
if(ENABLE_COMPONENT_UART)
  add_definitions(-DCONFIG_INTERFACE_TYPE_UART)
endif()
if(ENABLE_COMPONENT_I2C)
  add_definitions(-DCONFIG_INTERFACE_TYPE_I2C)
endif()

include_directories(include)
project(IonBridge)

idf_build_set_property(
  COMPILE_DEFINITIONS "dhcp_network_changed=dhcp_network_changed_link_up"
  APPEND)
