cmake_minimum_required(VERSION 3.15)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

project(ProgramCDFR LANGUAGES C CXX)

# C++20 on every target, so the x86_64 and aarch64 builds agree (libcamera's
# public headers need std::span on the Pi).
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(compiler_config)
include(dependencies)
include(CTest)

add_subdirectory(dependencies)
add_subdirectory(src)

# Deployment targets
include(deploy)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
