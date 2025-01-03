if(ENABLE_COMPONENT_UART)
  message(STATUS "Including Component UART")
  file(GLOB SRCS_FILES *.cpp)
else()
  message(STATUS "Skipping Component UART")
  set(SRCS_FILES "")
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  PRIV_REQUIRES
  driver
  power
  utils)
