add_library(i2c OBJECT)

target_sources(i2c
    PRIVATE
        src/Arduino.cpp
        src/drive_interface.cpp
        src/i2c.cpp
)

target_include_directories(i2c
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(i2c
    PUBLIC
        utils
        i2c_lib
        external_robot_interfaces
        project_compiler_options
)

add_library(ProgramCDFR::i2c ALIAS i2c)
