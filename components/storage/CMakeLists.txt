# Core storage implementation
set(SRCS_FILES storage.cpp)

# Conditionally add compression implementations based on environment variables
# (Kconfig values aren't available during first CMake pass)
if("$ENV{USE_GZIP_COMPRESSION}" STREQUAL "1")
  list(APPEND SRCS_FILES gzip_decompress.cpp)
endif()

if("$ENV{USE_XZ_COMPRESSION}" STREQUAL "1")
  list(APPEND SRCS_FILES xz_decompress.cpp)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  version
  PRIV_REQUIRES
  mbedtls
  littlefs)
