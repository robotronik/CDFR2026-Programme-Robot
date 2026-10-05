add_library(lidar OBJECT)

target_sources(lidar
    PRIVATE
        src/Lidar.cpp
        src/lidarAnalize.cpp
)

target_include_directories(lidar
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lidar
    PUBLIC
        rplidar
        defs
        i2c
        utils
)

add_library(ProgramCDFR::lidar ALIAS lidar)
