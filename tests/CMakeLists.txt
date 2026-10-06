add_executable(robot_tests
    main.cpp
    LidarTests.cpp
    LoggerTest.cpp
)

target_link_libraries(robot_tests PRIVATE
    robot_core
    project_compiler_options
)

# Register test with CTest, setting working directory to tests/ so relative test data paths resolve properly
add_test(NAME robot_tests COMMAND robot_tests)
set_tests_properties(robot_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
