add_library(navigation OBJECT)

target_sources(navigation
    PRIVATE
        src/astar.cpp
        src/driveControl.cpp
        src/navigation.cpp
        src/pathfind.cpp
)

target_include_directories(navigation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(navigation
    PUBLIC
        defs
        lidar
        vision
        utils
        external_robot_interfaces
        project_compiler_options
)

add_library(ProgramCDFR::navigation ALIAS navigation)
