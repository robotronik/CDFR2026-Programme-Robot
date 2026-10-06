cmake_minimum_required(VERSION 3.15)
project(ProgramCDFR2026 LANGUAGES CXX)

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
    GIT_TAG v1.2.0 # 
)
FetchContent_MakeAvailable(Crow)

# ------------------------------------------------------------------------------
# 1. Configuration Globale
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Forcer les couleurs
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
endif()

# Opti 1: CCache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# Opti 2: Linker rapide
if(NOT CMAKE_CROSSCOMPILING)
    find_program(MOLD_PATH NAMES mold)
    find_program(LLD_PATH NAMES ld.lld)

    if(MOLD_PATH)
        add_link_options("-fuse-ld=mold")
    elseif(LLD_PATH)
        add_link_options("-fuse-ld=lld")
    endif()
endif()

# Options de compilation de base
add_compile_options(-Wall -Wextra -g -O0)
add_compile_options("-fmacro-prefix-map=${CMAKE_SOURCE_DIR}/=")

# ------------------------------------------------------------------------------
# 2. Gestion du code source
# ------------------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES "src/*.cpp")
list(REMOVE_ITEM CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

# Le backend libcamera n'est compilé que pour ARM : le build local (x86_64) n'a
# pas libcamera et utilise V4L2 (cf. le bloc libcamera plus bas).
if(NOT CMAKE_CROSSCOMPILING)
    list(REMOVE_ITEM CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/vision/LibcameraCamera.cpp")
endif()

# Création de la librairie contenant tout le code
add_library(robot_core STATIC ${CORE_SOURCES})

# ------------------------------------------------------------------------------
# 3. Includes
# ------------------------------------------------------------------------------
target_include_directories(robot_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../CDFR2026-Program-DriveControl/include/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/../cdfr2024-programme-Actionneur/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/WiringPi/wiringPi
)

# ------------------------------------------------------------------------------
# 4. Librairies externes
# ------------------------------------------------------------------------------

# SQLite : base des durées d'action (cf. src/db/ActionDurationDB.cpp).
#
# FindSQLite3 de CMake cherche la lib dans des chemins multiarch (lib/<triplet>)
# qui peuvent disparaître de la recherche selon l'état du cache, d'où un
# "Could NOT find SQLite3 (missing: SQLite3_LIBRARY)" alors que la lib est bien
# installée (cas vécu localement, alors qu'un configure neuf fonctionne).
# pkg-config fournit le libdir de façon fiable : on l'utilise en priorité en
# local, avec repli sur le module CMake. On expose dans tous les cas le target
# SQLite::SQLite3 pour ne rien changer à la ligne de link. En cross-compilation
# on garde find_package, qui respecte CMAKE_FIND_ROOT_PATH (cf. pi_toolchain).
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

# OpenCV : détection native des marqueurs ArUco (cf. src/vision/ArucoDetector.cpp).
# Sur Debian/Ubuntu le config vit sous lib/<arch>/cmake/opencv4, que la recherche
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
target_include_directories(robot_core PUBLIC ${OpenCV_INCLUDE_DIRS})

# libcamera : capture native sur Raspberry Pi 5. Le pipeline V4L2 n'expose que
# des trames Bayer brutes (l'ISP PiSP est en amont), que le détecteur ArUco ne
# sait pas décoder ; libcamera pilote capteur -> ISP -> trames utilisables.
# Sélectionné automatiquement pour la compilation croisée ARM, le backend V4L2
# restant utilisé en local (x86_64) pour que les tests tournent sans libcamera
# installée (cf. pi_toolchain.cmake).
if(CMAKE_CROSSCOMPILING)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
    if(NOT TARGET PkgConfig::LIBCAMERA)
        message(FATAL_ERROR
            "libcamera introuvable en compilation croisée. Exécutez "
            "./scripts/fetch_arm64_sysroot.sh pour l'ajouter au sysroot ARM64, "
            "puis relancez ./build.sh build_arm.")
    endif()
    target_link_libraries(robot_core PUBLIC PkgConfig::LIBCAMERA)
    target_compile_definitions(robot_core PUBLIC CAMERA_USE_LIBCAMERA)

    # Debian/Ubuntu et Raspberry Pi OS ne livrent pas base/event_dispatcher.h ni
    # base/thread.h dans libcamera-dev, alors que le code de capture en a besoin
    # pour faire tourner la boucle d'événements. On ajoute des en-têtes de
    # compatibilité maison uniquement lorsque les officiels manquent.
    find_path(LIBCAMERA_EVENT_DISPATCHER_HEADER
        NAMES libcamera/base/event_dispatcher.h
        PATHS ${LIBCAMERA_INCLUDE_DIRS}
        NO_DEFAULT_PATH)
    if(NOT LIBCAMERA_EVENT_DISPATCHER_HEADER)
        target_include_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/compat)
        message(STATUS
            "libcamera/base/event_dispatcher.h absent : ajout des en-têtes de compatibilité maison")
    endif()

    # libcamera 0.2+ regroupe les formats XRGB/ARGB dans le type PixelFormat à
    # la place des anciennes classes ; les tests de format du convertisseur
    # utilisent ce garde.
    if(LIBCAMERA_VERSION VERSION_GREATER_EQUAL "0.2")
        target_compile_definitions(robot_core PUBLIC CAMERA_LIBCAMERA_HAVE_XRGB8888)
    endif()
    # Les formats 24 bits RGB/BGR ont été renommés RGB24/BGR24 en libcamera 0.3
    # (RGB888/BGR888 en 0.2) : le convertisseur s'adapte via ce garde.
    if(LIBCAMERA_VERSION VERSION_GREATER_EQUAL "0.3")
        target_compile_definitions(robot_core PUBLIC CAMERA_LIBCAMERA_FORMAT_RGB24)
    endif()
endif()
target_link_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/output/Linux/Release)

if(CMAKE_CROSSCOMPILING)
    target_link_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64-linux-gnu)
    # Le sysroot arm64 (cf. scripts/fetch_arm64_sysroot.sh) ne contient qu'OpenCV
    # et SQLite, pas les dépendances propres des modules OpenCV (libpng, libtiff,
    # libavcodec, GStreamer, VTK...). En compilation croisée l'éditeur de liens
    # exige par défaut que les symboles non résolus des .so liées soient
    # satisfaits par les bibliothèques de la ligne de commande ; ces symboles
    # seront résolus à l'exécution sur la Raspberry Pi, d'où l'option ci-dessous.
    # Les symboles non résolus de nos propres objets restent, eux, des erreurs.
    target_link_options(robot_core INTERFACE -Wl,--allow-shlib-undefined)
else()
    target_link_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64-linux-gnu)
endif()

target_link_libraries(robot_core PUBLIC Crow::Crow pthread i2c rt sl_lidar_sdk SQLite::SQLite3 ${OpenCV_LIBS})

# ------------------------------------------------------------------------------
# 5. Cible 1 : L'exécutable Principal
# ------------------------------------------------------------------------------
add_executable(programCDFR src/main.cpp)
target_link_libraries(programCDFR PRIVATE robot_core)

# ------------------------------------------------------------------------------
# 6. Cible 2 : Les Tests
# ------------------------------------------------------------------------------
file(GLOB_RECURSE TEST_SOURCES "tests/*.cpp")
add_executable(robot_tests ${TEST_SOURCES})
target_link_libraries(robot_tests PRIVATE robot_core)

add_custom_command(TARGET robot_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/tests/data $<TARGET_FILE_DIR:robot_tests>/tests/data
)

# ------------------------------------------------------------------------------
# 7. Copie des fichiers annexes (Post-Build)
# ------------------------------------------------------------------------------
add_custom_command(TARGET programCDFR POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/html $<TARGET_FILE_DIR:programCDFR>/html
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/tests/lidar $<TARGET_FILE_DIR:programCDFR>/tests/lidar
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/data $<TARGET_FILE_DIR:programCDFR>/data
)
