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

# Options de compilation de base
add_compile_options(-Wall -Wextra -g -O0)

# ------------------------------------------------------------------------------
# 2. Gestion du code source
# ------------------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES "src/*.cpp")
list(REMOVE_ITEM CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

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

target_link_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/output/Linux/Release)

if(CMAKE_CROSSCOMPILING)
    target_link_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64-linux-gnu)
else()
    target_link_directories(robot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64-linux-gnu)
endif()

target_link_libraries(robot_core PUBLIC Crow::Crow pthread i2c rt sl_lidar_sdk)

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

# ------------------------------------------------------------------------------
# 7. Copie des fichiers annexes (Post-Build)
# ------------------------------------------------------------------------------
add_custom_command(TARGET programCDFR POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/html $<TARGET_FILE_DIR:programCDFR>/html
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/tests/lidar $<TARGET_FILE_DIR:programCDFR>/tests/lidar
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../PythonArucoOpenCV/data $<TARGET_FILE_DIR:programCDFR>/data
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../PythonArucoOpenCV/pi_detect_aruco.py $<TARGET_FILE_DIR:programCDFR>/
)
