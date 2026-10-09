cmake_minimum_required(VERSION 3.15)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

project(ProgramCDFR LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
