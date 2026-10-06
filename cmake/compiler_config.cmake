add_library(project_compiler_options INTERFACE)

target_compile_features(project_compiler_options INTERFACE cxx_std_17)

target_compile_options(project_compiler_options INTERFACE
    -Wall
    -Wextra
    "-fmacro-prefix-map=${CMAKE_SOURCE_DIR}/="
    # Compiler diagnostic colors
    $<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>
    $<$<CXX_COMPILER_ID:Clang>:-fcolor-diagnostics>
)

# Output binaries directly into the build root for straightforward execution and packaging
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}" CACHE PATH "Runtime output directory")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" CACHE PATH "Library output directory")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" CACHE PATH "Archive output directory")

if(NOT CMAKE_CROSSCOMPILING)
    find_program(MOLD_PATH NAMES mold)
    find_program(LLD_PATH NAMES ld.lld)

    if(MOLD_PATH)
        target_link_options(project_compiler_options INTERFACE "-fuse-ld=mold")
    elseif(LLD_PATH)
        target_link_options(project_compiler_options INTERFACE "-fuse-ld=lld")
    endif()
else()
    # Le sysroot ARM64 (cf. scripts/fetch_arm64_sysroot.sh) ne contient que les
    # paquets que l'on y extrait explicitement (OpenCV, SQLite, libcamera), pas
    # leurs dépendances transitives (libpng, libtiff, ...). À l'édition de liens
    # ces symboles seront résolus sur la Raspberry Pi ; sans cette option
    # l'éditeur de liens échoue en déclarant introuvables les bibliothèques
    # NEEDED des .so du sysroot.
    target_link_options(project_compiler_options INTERFACE "-Wl,--allow-shlib-undefined")
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# Automatically link compile_commands.json into the project root for LSP servers (clangd, etc.)
if(CMAKE_EXPORT_COMPILE_COMMANDS AND NOT CMAKE_CROSSCOMPILING)
    if(EXISTS "${CMAKE_SOURCE_DIR}/compile_commands.json" OR IS_SYMLINK "${CMAKE_SOURCE_DIR}/compile_commands.json")
        file(REMOVE "${CMAKE_SOURCE_DIR}/compile_commands.json")
    endif()
    file(CREATE_LINK
        "${CMAKE_BINARY_DIR}/compile_commands.json"
        "${CMAKE_SOURCE_DIR}/compile_commands.json"
        SYMBOLIC
    )
endif()
