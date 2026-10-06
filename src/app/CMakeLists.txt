add_executable(programCDFR main.cpp)

target_link_libraries(programCDFR PRIVATE
    robot_core
    project_compiler_options
)

add_custom_command(TARGET programCDFR POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/html $<TARGET_FILE_DIR:programCDFR>/html
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/tests/lidar $<TARGET_FILE_DIR:programCDFR>/tests/lidar
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CDFR_EXTERNAL_DIR}/PythonArucoOpenCV/data $<TARGET_FILE_DIR:programCDFR>/data
    COMMAND ${CMAKE_COMMAND} -E copy ${CDFR_EXTERNAL_DIR}/PythonArucoOpenCV/pi_detect_aruco.py $<TARGET_FILE_DIR:programCDFR>/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/autoRunInstaller.sh $<TARGET_FILE_DIR:programCDFR>/
)
