add_library(defs OBJECT)

target_sources(defs
    PRIVATE
        src/structs.cpp
        src/tableState.cpp
)

target_include_directories(defs
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../navigation/include
)

target_link_libraries(defs
    PUBLIC
        utils
        external_robot_interfaces
        project_compiler_options
)
