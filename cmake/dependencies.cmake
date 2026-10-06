#-----------------------------------------
# Dependency fetcher
#-----------------------------------------

include(FetchContent)

# Asio
FetchContent_Declare(
    asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
    GIT_TAG asio-1-30-2
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(asio)

set(ASIO_INCLUDE_DIR ${asio_SOURCE_DIR}/asio/include CACHE PATH "Asio include dir" FORCE)

# Crow C++ Framework
FetchContent_Declare(
    Crow
    GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
    GIT_TAG v1.2.0
    GIT_SHALLOW TRUE
    GIT_SUBMODULES ""
)
FetchContent_MakeAvailable(Crow)

#-----------------------------------------
# System dependencies
#-----------------------------------------

find_package(Threads REQUIRED)

find_package(SQLite3 REQUIRED)

#-----------------------------------------
# External robot interfaces
#-----------------------------------------
# Provided by the build images (docker/Dockerfile.*); overridable for custom setups.
set(CDFR_EXTERNAL_DIR "/opt/cdfr" CACHE PATH "Location of the external CDFR robot repositories")

add_library(external_robot_interfaces INTERFACE)
target_include_directories(external_robot_interfaces INTERFACE
    ${CDFR_EXTERNAL_DIR}/CDFR2026-Program-DriveControl/include/interface
    ${CDFR_EXTERNAL_DIR}/cdfr2024-programme-Actionneur/include/common
)
