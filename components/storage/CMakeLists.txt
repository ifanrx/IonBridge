file(GLOB SRCS_FILES *.cpp)
idf_component_register(SRCS ${SRCS_FILES} INCLUDE_DIRS "include" PRIV_REQUIRES mbedtls )
