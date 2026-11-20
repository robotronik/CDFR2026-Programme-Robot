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
    private:
        int wall1Tab[3] = {PUMP_1_NUM, 0, 0};
        int wall2Tab[3] = {0, 0, 0};
        int wall3Tab[3] = {0, 0, 0};
        int wall4Tab[3] = {0, 0, 0};

        int gate1Tab[3] = {0, 0, 0};
        int gate2Tab[3] = {0, 0, 0};
};
