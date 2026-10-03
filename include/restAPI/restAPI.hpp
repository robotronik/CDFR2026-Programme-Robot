#pragma once

#include "defs/mainState.hpp"

class DriveControl;
class TableState;
class Arduino;
class Lidar;
class ArucoCam;

class RestAPI {
    public:
        RestAPI(main_State_t* currentState,
                main_State_t* nextState,
                DriveControl* drive,
                TableState* tableStatus,
                Arduino* arduino,
                Lidar* lidar,
                ArucoCam* arucoCam);
        ~RestAPI() = default;

        void start();
        void stop();

    private:
        main_State_t* currentState;
        main_State_t* nextState;
        DriveControl* drive;
        TableState* tableStatus;
        Arduino* arduino;
        Lidar* lidar;
        ArucoCam* arucoCam;
};