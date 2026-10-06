add_library(utils OBJECT)

target_sources(utils
    PRIVATE
        src/logger.cpp
)

target_include_directories(utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(utils
    PUBLIC
        project_compiler_options
)
