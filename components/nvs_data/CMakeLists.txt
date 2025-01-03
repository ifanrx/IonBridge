file(GLOB_RECURSE SOURCES "*.cpp")
idf_component_register(
  SRCS
  ${SOURCES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  nvs_flash
  PRIV_REQUIRES
  )
