add_library(defs OBJECT)

target_sources(defs
    PRIVATE
        src/structs.cpp
        src/tableState.cpp
)

target_include_directories(defs
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(defs
    PUBLIC
        utils
        navigation
)

add_library(ProgramCDFR::defs ALIAS defs)
