add_library(project_compiler_options INTERFACE)

target_compile_features(project_compiler_options INTERFACE cxx_std_17)

target_compile_options(project_compiler_options INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    "-fmacro-prefix-map=${CMAKE_SOURCE_DIR}/="
    # Couleurs
    $<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>
    $<$<CXX_COMPILER_ID:Clang>:-fcolor-diagnostics>
)

if(NOT CMAKE_CROSSCOMPILING)
    find_program(MOLD_PATH NAMES mold)
    find_program(LLD_PATH NAMES ld.lld)

    if(MOLD_PATH)
        target_link_options(project_compiler_options INTERFACE "-fuse-ld=mold")
    elseif(LLD_PATH)
        target_link_options(project_compiler_options INTERFACE "-fuse-ld=lld")
    endif()
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()
