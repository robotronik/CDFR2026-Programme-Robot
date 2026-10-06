add_library(vision OBJECT)

target_sources(vision
    PRIVATE
        src/ArucoCam.cpp
        src/ransac.cpp
)

target_include_directories(vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(vision
    PUBLIC
        utils
        project_compiler_options
)
