cmake_minimum_required(VERSION 3.15)

include(ExternalProject)

set(SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk")
set(OUTPUT_LIB    "${SUBMODULE_DIR}/rplidar_sdk.a")

ExternalProject_Add(
    rplidar_make
    SOURCE_DIR        "${SUBMODULE_DIR}"
    DOWNLOAD_COMMAND  ""
    UPDATE_COMMAND    ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND     make -C <SOURCE_DIR> CC=${CMAKE_C_COMPILER}
    INSTALL_COMMAND   ""
    BUILD_IN_SOURCE   TRUE
    BUILD_BYPRODUCTS  "${OUTPUT_LIB}"
)

add_library(rplidar_sdk STATIC IMPORTED GLOBAL)
set_target_properties(rplidar_sdk PROPERTIES
    IMPORTED_LOCATION "${OUTPUT_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${SUBMODULE_DIR}/include"
)
add_dependencies(rplidar_sdk rplidar_make)
