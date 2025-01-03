file(GLOB SRCS_FILES *.cpp)
idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  PRIV_REQUIRES
  animation
  app
  nvs_data
  utils
  wifi)
