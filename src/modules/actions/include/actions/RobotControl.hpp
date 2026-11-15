#pragma once
#include "actions/ActuatorsControl.hpp"
#include "defs/constante.h"

class RobotControl {
    public:
        RobotControl(ActuatorsControl* actuators, DriveControl* drive);
        Wall wall1;
        Wall wall2;
        Wall wall3;
        Wall wall4;
        Door door1;
        Door door2;
    private:
        int wall1Tab[3] = {PUMP_1_NUM, 0, 0};
        int wall2Tab[3] = {0, 0, 0};
        int wall3Tab[3] = {0, 0, 0};
        int wall4Tab[3] = {0, 0, 0};

        int door1Tab[3] = {0, 0, 0};
        int door2Tab[3] = {0, 0, 0};
        ActuatorsControl* actuators;
        DriveControl* drive;
};