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

# Register test with CTest, running from the project root so the shared relative
# test-data paths (tests/data, data, tests/lidar) resolve.
add_test(NAME robot_tests COMMAND robot_tests)
set_tests_properties(robot_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)
