add_subdirectory(utils)
add_subdirectory(db)
add_subdirectory(mat)
add_subdirectory(vision)
add_subdirectory(defs)
add_subdirectory(i2c)
add_subdirectory(lidar)
add_subdirectory(navigation)
add_subdirectory(actions)
add_subdirectory(restAPI)

# Combined static library for the entire robot software
add_library(robot_core STATIC)

target_link_libraries(robot_core PUBLIC
    utils
    db
    mat
    vision
    defs
    i2c
    lidar
    navigation
    actions
    restAPI
    project_compiler_options
)
