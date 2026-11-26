#pragma once
#include "actions/ActuatorsControl.hpp"
#include "defs/constante.h"
#include "navigation/driveControl.h"

class Arduino;

/*
    Point d'entrée unique sur le matériel du robot : possède le contrôle du
    châssis (DriveControl) et des actionneurs (ActuatorsControl), et expose les
    éléments de jeu construits par le robot. Les modules qui ont besoin du
    matériel ne reçoivent que cette instance.
*/
class RobotControl {
    private:
        Gate::actuatorsIdx_t idxWall1{
            .pumpIdx = PUMP_1_NUM,
            .electroIdx = {PUMP_2_NUM, 0, 0},
            .servoIdx = {0, 0, 0}
        };
        Gate::actuatorsIdx_t idxWall2{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {SERVO_CLAW_CLOSE_1, 0, 0}
        };
        Gate::actuatorsIdx_t idxWall3{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {0, 0, 0}
        };
        Gate::actuatorsIdx_t idxWall4{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {0, 0, 0}
        };
        Gate::actuatorsIdx_t idxGate1{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {2, 3, 0}
        };
        Gate::actuatorsIdx_t idxGate2{
            .pumpIdx = 0,
            .electroIdx = {0, 0, 0},
            .servoIdx = {4, 5, 0}
        };
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

        Wall* wallTable[6];
        void homeActuators();
    
};
