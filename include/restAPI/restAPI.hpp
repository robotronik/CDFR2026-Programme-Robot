#pragma once

#include <string>
#include <vector>

#include "defs/mainState.hpp"
#include "defs/structs.hpp"

class DriveControl;
class TableState;
class Arduino;
class Lidar;
class ArucoCam;
class VirtualStrategy;

class RestAPI {
    public:
        RestAPI(main_State_t* currentState,
                main_State_t* nextState,
                DriveControl* drive,
                TableState* tableStatus,
                Arduino* arduino,
                Lidar* lidar,
                ArucoCam* arucoCam,
                std::vector<VirtualStrategy*>* strategies);
        ~RestAPI() = default;

        void start();
        void stop();

        // Requests received through the REST API, applied by the main loop
        bool consumeColorRequest(colorTeam_t& color);
        bool consumeStrategyRequest(std::string& strategy);

    private:
        main_State_t* currentState;
        main_State_t* nextState;
        DriveControl* drive;
        TableState* tableStatus;
        Arduino* arduino;
        Lidar* lidar;
        ArucoCam* arucoCam;
        std::vector<VirtualStrategy*>* strategies;

        bool hasColorRequest = false;
        colorTeam_t requestedColor = NONE;
        bool hasStrategyRequest = false;
        std::string requestedStrategy;
};