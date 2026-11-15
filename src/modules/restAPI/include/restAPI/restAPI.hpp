#pragma once

#include <string>
#include <vector>

#include "defs/mainState.hpp"
#include "defs/structs.hpp"

class DriveControl;
class TableState;
class Arduino;
class Lidar;
class Cam;
class Navigation;
class VirtualStrategy;
class SensorControl;
class ActuatorsControl;

class RestAPI {
    public:
        RestAPI(MainState* currentState,
                MainState* nextState,
                DriveControl* drive,
                TableState* tableStatus,
                Arduino* arduino,
                SensorControl* sensor,
                ActuatorsControl* actuators,
                Lidar* lidar,
                Cam* cam,
                Navigation* navigation,
                std::vector<VirtualStrategy*>* strategies);
        ~RestAPI() = default;

        void start();
        void stop();

        // Requests received through the REST API, applied by the main loop
        bool consumeColorRequest(ColorTeam& color);
        bool consumeStrategyRequest(std::string& strategy);

    private:
        MainState* currentState;
        MainState* nextState;
        DriveControl* drive;
        TableState* tableStatus;
        Arduino* arduino;
        SensorControl* sensor;
        ActuatorsControl* actuators;
        Lidar* lidar;
        Cam* cam;
        Navigation* navigation;
        std::vector<VirtualStrategy*>* strategies;

        bool hasColorRequest = false;
        ColorTeam requestedColor = ColorTeam::None;
        bool hasStrategyRequest = false;
        std::string requestedStrategy;
};