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

# The MAT (robot vision server) is enabled for the ARM cross-build and disabled
# for local x86_64 builds; override with -DCDFR_ENABLE_MAT=ON/OFF.
if(CMAKE_CROSSCOMPILING)
    set(_mat_default ON)
else()
    set(_mat_default OFF)
endif()
option(CDFR_ENABLE_MAT "Connect to the MAT vision server" ${_mat_default})

if(NOT CDFR_ENABLE_MAT)
    target_compile_definitions(mat PUBLIC DISABLE_MAT)
endif()
