add_executable(robot_tests
    main.cpp
    LidarTests.cpp
    LoggerTest.cpp
    MatTest.cpp
    test_aruco.cpp
    test_features.cpp
)

target_link_libraries(robot_tests PRIVATE
    robot_core
    project_compiler_options
)

# One test per group. Tests run from the project root so the shared relative
# test-data paths (tests/data, data, tests/lidar) resolve.
foreach(group log lidar mat aruco features)
    add_test(NAME ${group} COMMAND robot_tests ${group})
    set_tests_properties(${group} PROPERTIES
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
endforeach()
