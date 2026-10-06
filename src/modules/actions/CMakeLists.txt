add_library(actions OBJECT)

target_sources(actions
    PRIVATE
        src/ElementalAction/CalibrationAction.cpp
        src/ElementalAction/GoToPositionAction.cpp
        src/ElementalAction/NavHomeAction.cpp
        src/ElementalAction/WaitAction.cpp
        src/Strategy/ExempleStrategy.cpp
        src/ActuatorsControl.cpp
        src/MainActionFSM.cpp
        src/SensorControl.cpp
        src/calibration.cpp
)

target_include_directories(actions
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(actions
    PUBLIC
        navigation
        db
        defs
        i2c
        vision
        utils
        external_robot_interfaces
        project_compiler_options
)

add_library(ProgramCDFR::actions ALIAS actions)
