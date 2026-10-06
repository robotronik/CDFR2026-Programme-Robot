add_executable(programCDFR main.cpp)

target_include_directories(programCDFR PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(programCDFR PRIVATE
    robot_core
    project_compiler_options
)

add_custom_command(TARGET programCDFR POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/html $<TARGET_FILE_DIR:programCDFR>/html
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/tests/lidar $<TARGET_FILE_DIR:programCDFR>/tests/lidar
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/data $<TARGET_FILE_DIR:programCDFR>/data
)
