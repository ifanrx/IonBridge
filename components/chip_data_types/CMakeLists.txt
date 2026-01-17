include(${PROJECT_DIR}/component_requires.cmake)
if(ENABLE_COMPONENT_UART)
  file(GLOB SRC_FILES "*.cpp")
  idf_component_register(
    SRCS
    ${SRC_FILES}
    INCLUDE_DIRS
    "include"
    REQUIRES
    nvs_data
    rpc)
else()
  idf_component_register(INCLUDE_DIRS "include")
endif()
