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

target_link_libraries(PUBLIC
    utils
    navigation
)

add_library(ProgramCDFR::i2c ALIAS i2c)
