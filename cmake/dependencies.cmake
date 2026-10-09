#-----------------------------------------
# Dependency fetcher (header-only)
#-----------------------------------------

include(FetchContent)

# Asio is the networking backend of Crow, the HTTP/WebSocket framework behind the
# REST API. Both are header-only, so they are wired up as INTERFACE targets
# directly: this skips Crow's CMakeLists (and with it its optional Python3
# probe) and keeps the build free of any Python dependency.
FetchContent_Declare(
    asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
    GIT_TAG asio-1-30-2
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR unused
)
FetchContent_Declare(
    Crow
    GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
    GIT_TAG v1.2.0
    GIT_SHALLOW TRUE
    GIT_SUBMODULES ""
    SOURCE_SUBDIR unused
)
FetchContent_MakeAvailable(asio Crow)

add_library(asio INTERFACE)
target_include_directories(asio INTERFACE "${asio_SOURCE_DIR}/asio/include")
add_library(asio::asio ALIAS asio)

add_library(Crow INTERFACE)
target_include_directories(Crow INTERFACE "${crow_SOURCE_DIR}/include")
target_link_libraries(Crow INTERFACE asio::asio)
add_library(Crow::Crow ALIAS Crow)

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
