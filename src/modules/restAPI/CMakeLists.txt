add_library(restAPI OBJECT)

target_sources(restAPI
    PRIVATE
        src/restAPI.cpp
        src/matAPI.cpp
        src/manual_mode.cpp
)

target_include_directories(restAPI
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(restAPI
    PUBLIC
        defs
        i2c
        lidar
        navigation
        actions
        vision
        utils
        Crow::Crow
        Threads::Threads
        project_compiler_options
)
