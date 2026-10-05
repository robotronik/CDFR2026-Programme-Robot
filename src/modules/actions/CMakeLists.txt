add_library(actions OBJECT)

target_sources(actions
    PRIVATE
        src/ElementalAction/CalibrationAction.cpp
        src/ElementalAction/GoToPositionAction.cpp
        src/Strategy/ExampleStrategy.cpp
        src/NavHomeAction.cpp
        src/WaitAction.cpp
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
)

add_library(ProgramCDFR::actions ALIAS actions)
