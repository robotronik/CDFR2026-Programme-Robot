add_executable(robot_program main.cpp)

target_link_libraries(robot_program PRIVATE robot_core)

add_custom_command(TARGET programCDFR POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/html $<TARGET_FILE_DIR:programCDFR>/html
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/tests/lidar $<TARGET_FILE_DIR:programCDFR>/tests/lidar
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../PythonArucoOpenCV/data $<TARGET_FILE_DIR:programCDFR>/data
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../PythonArucoOpenCV/pi_detect_aruco.py $<TARGET_FILE_DIR:programCDFR>/
)
