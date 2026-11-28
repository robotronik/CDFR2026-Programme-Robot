#pragma once
#include "actions/ActuatorsControl.hpp"
#include "defs/constante.h"
#include "navigation/driveControl.h"

class Arduino;

/*
    Single entry point to the robot's hardware: owns the chassis (DriveControl)
    and actuator (ActuatorsControl) control, and exposes the game elements built
    by the robot. Modules that need the hardware only receive this instance.
*/
class RobotControl {
    private:
        Gate::actuatorsIdx_t idxWall1{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {0, 0, 0}};
        Gate::actuatorsIdx_t idxWall2{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {SERVO_CLAW_CLOSE_1, 0, 0}};
        Gate::actuatorsIdx_t idxWall3{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {0, 0, 0}};
        Gate::actuatorsIdx_t idxWall4{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {0, 0, 0}};
        Gate::actuatorsIdx_t idxGate1{
            .pumpIdx = PUMP_1_NUM,
            .electroIdx = {PUMP_2_NUM, 0, 0},
            .servoIdx = {2, 3, 0}};
        Gate::actuatorsIdx_t idxGate2{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {4, 5, 0}};

    public:
        explicit RobotControl(Arduino* arduino);

        DriveControl drive;
        ActuatorsControl actuators;

        Wall wall1;
        Wall wall2;
        Wall wall3;
        Wall wall4;
        Gate gate1;
        Gate gate2;

        Wall* wallTable[2];
        bool homeActuators();
};
