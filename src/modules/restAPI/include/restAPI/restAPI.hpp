#pragma once

#include <chrono>
#include <memory>
#include <mutex>
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

struct SimulationRequest {
    enum Type { None, Start, Reset };
    Type type = None;
    colorTeam_t color = NONE;
    VirtualStrategy* strategy = nullptr;
};

class RestAPI {
    public:
        RestAPI(main_State_t* currentState,
                main_State_t* nextState,
                DriveControl* drive,
                TableState* tableStatus,
                Arduino* arduino,
                Lidar* lidar,
                Cam* cam,
                Navigation* navigation,
                std::vector<VirtualStrategy*>* strategies);
        ~RestAPI();

        void start(bool simulation);
        void stop();
        void publishSimulation();
        SimulationRequest consumeSimulationRequest();

        // Requests received through the REST API, applied by the main loop
        bool consumeColorRequest(colorTeam_t& color);
        bool consumeStrategyRequest(std::string& strategy);

    private:
        void registerRobotRoutes(bool simulation);
        void registerRobotCommands();
        void expireSimulationView();
        json readSimulationSnapshot();
        json strategyNames() const;
        VirtualStrategy* findStrategy(const std::string& name) const;

        main_State_t* currentState;
        main_State_t* nextState;
        DriveControl* drive;
        TableState* tableStatus;
        Arduino* arduino;
        Lidar* lidar;
        Cam* cam;
        Navigation* navigation;
        std::vector<VirtualStrategy*>* strategies;
        std::unique_ptr<VirtualStrategy> explorationStrategy;
        std::unique_ptr<VirtualStrategy> stocksStrategy;

        std::mutex simulationMutex;
        SimulationRequest simulationRequest;
        std::chrono::steady_clock::time_point simulationViewUntil{};
        bool simulationNeedsView = false;
        bool simulationReady = false;
        json simulationSnapshot;

        bool hasColorRequest = false;
        colorTeam_t requestedColor = NONE;
        bool hasStrategyRequest = false;
        std::string requestedStrategy;
};
