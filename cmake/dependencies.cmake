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

# SQLite3 dependency (uses system package when available, or builds official amalgamation when cross-compiling)
find_package(SQLite3 QUIET)
if(SQLite3_FOUND)
    if(TARGET SQLite::SQLite3 AND NOT TARGET SQLite3::SQLite3)
        add_library(SQLite3::SQLite3 ALIAS SQLite::SQLite3)
    elseif(TARGET SQLite3::SQLite3 AND NOT TARGET SQLite::SQLite3)
        add_library(SQLite::SQLite3 ALIAS SQLite3::SQLite3)
    endif()
else()
    message(STATUS "SQLite3 not found in sysroot, fetching amalgamation...")
    if(POLICY CMP0169)
        cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Declare(
        sqlite3_amalgamation
        URL
            https://www.sqlite.org/2024/sqlite-amalgamation-3450100.zip
            http://www.sqlite.org/2024/sqlite-amalgamation-3450100.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_GetProperties(sqlite3_amalgamation)
    if(NOT sqlite3_amalgamation_POPULATED)
        FetchContent_Populate(sqlite3_amalgamation)
        add_library(sqlite3_bundled STATIC "${sqlite3_amalgamation_SOURCE_DIR}/sqlite3.c")
        target_include_directories(sqlite3_bundled PUBLIC "${sqlite3_amalgamation_SOURCE_DIR}")
        target_link_libraries(sqlite3_bundled PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
        add_library(SQLite3::SQLite3 ALIAS sqlite3_bundled)
        add_library(SQLite::SQLite3 ALIAS sqlite3_bundled)
    endif()
endif()

#-----------------------------------------
# External robot interfaces
#-----------------------------------------
add_library(external_robot_interfaces INTERFACE)
target_include_directories(external_robot_interfaces INTERFACE
    ${CMAKE_SOURCE_DIR}/../CDFR2026-Program-DriveControl/include/interface
    ${CMAKE_SOURCE_DIR}/../cdfr2024-programme-Actionneur/include/common
)
