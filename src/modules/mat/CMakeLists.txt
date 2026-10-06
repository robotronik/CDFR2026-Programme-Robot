add_library(mat OBJECT)

target_sources(mat
    PRIVATE
        src/mat.cpp
)

target_include_directories(mat
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mat
    PUBLIC
        utils
        project_compiler_options
)
