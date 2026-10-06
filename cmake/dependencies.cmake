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

# SQLite : base des durées d'action (cf. src/modules/db/src/ActionDurationDB.cpp).
# FindSQLite3 de CMake cherche la lib dans des chemins multiarch (lib/<triplet>)
# qui peuvent disparaître de la recherche selon l'état du cache, d'où un
# "Could NOT find SQLite3 (missing: SQLite3_LIBRARY)" alors que la lib est bien
# installée. pkg-config fournit le libdir de façon fiable : on l'utilise en
# priorité en local, avec repli sur le module CMake. En cross-compilation on
# garde find_package, qui respecte CMAKE_FIND_ROOT_PATH (cf. le toolchain).
if(NOT CMAKE_CROSSCOMPILING)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(SQLite3 QUIET IMPORTED_TARGET sqlite3)
    endif()
    if(TARGET PkgConfig::SQLite3)
        add_library(SQLite::SQLite3 INTERFACE IMPORTED)
        target_link_libraries(SQLite::SQLite3 INTERFACE PkgConfig::SQLite3)
    endif()
endif()

if(NOT TARGET SQLite::SQLite3)
    find_package(SQLite3 REQUIRED)
endif()
# For legacy build systems
if(NOT TARGET SQLite3::SQLite3)
    add_library(SQLite3::SQLite3 ALIAS SQLite::SQLite3)
endif()

#-----------------------------------------
# OpenCV
#-----------------------------------------
# Détection native des marqueurs ArUco (cf. src/modules/vision/). Sur
# Debian/Ubuntu le config vit sous lib/<arch>/cmake/opencv4, que la recherche
# par défaut de CMake ne résout pas toujours (notamment sur les runners CI) : on
# le localise explicitement pour renseigner OpenCV_DIR.
if(NOT DEFINED OpenCV_DIR AND NOT CMAKE_CROSSCOMPILING)
    file(GLOB _opencv_config_dirs
        "/usr/lib/*/cmake/opencv4"
        "/usr/lib/cmake/opencv4"
        "/usr/local/lib/cmake/opencv4"
        "/usr/include/opencv4/")
    if(_opencv_config_dirs)
        list(GET _opencv_config_dirs 0 _opencv_config_dir)
        set(OpenCV_DIR "${_opencv_config_dir}" CACHE PATH "Répertoire du config CMake OpenCV")
        message(STATUS "OpenCV config trouvé dans ${OpenCV_DIR}")
    endif()
endif()
find_package(OpenCV REQUIRED CONFIG)

add_library(opencv_deps INTERFACE)
target_include_directories(opencv_deps INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv_deps INTERFACE ${OpenCV_LIBS})

#-----------------------------------------
# External robot interfaces
#-----------------------------------------
add_library(external_robot_interfaces INTERFACE)
target_include_directories(external_robot_interfaces INTERFACE
    ${CMAKE_SOURCE_DIR}/../CDFR2026-Program-DriveControl/include/interface
    ${CMAKE_SOURCE_DIR}/../cdfr2024-programme-Actionneur/include/common
)
