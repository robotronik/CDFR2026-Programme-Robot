#-----------------------------------------
# Dependency fetcher
#-----------------------------------------
# CMake-native libraries should be fetched in this way, others belong in
# the dependencies/ folder

include(FetchContent)
FetchContent_Declare(
    asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
    GIT_TAG asio-1-30-2
)
FetchContent_MakeAvailable(asio)

set(ASIO_INCLUDE_DIR ${asio_SOURCE_DIR}/asio/include CACHE PATH "Asio include dir" FORCE)

FetchContent_Declare(
    Crow
    GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
    GIT_TAG v1.2.0
)
FetchContent_MakeAvailable(Crow)

#-----------------------------------------
# System dependencies
#-----------------------------------------

find_package(SQLite3 REQUIRED)

#-----------------------------------------
# External includes
#-----------------------------------------
target_include_directories(robot_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../CDFR2026-Program-DriveControl/include/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/../cdfr2024-programme-Actionneur/include/common
)




