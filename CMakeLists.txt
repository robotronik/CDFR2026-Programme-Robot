cmake_minimum_required(VERSION 3.15)
project(ProgramCDFR2026 LANGUAGES CXX)
find_package(Crow)

# ------------------------------------------------------------------------------
# 1. Configuration Globale
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
)
