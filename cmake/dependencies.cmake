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
# For legacy build systems
if(NOT TARGET SQLite3::SQLite3)
    add_library(SQLite3::SQLite3 ALIAS SQLite::SQLite3)
endif()

#-----------------------------------------
# External robot interfaces
#-----------------------------------------
add_library(external_robot_interfaces INTERFACE)
target_include_directories(external_robot_interfaces INTERFACE
    ${CMAKE_SOURCE_DIR}/../CDFR2026-Program-DriveControl/include/interface
    ${CMAKE_SOURCE_DIR}/../cdfr2024-programme-Actionneur/include/common
)
