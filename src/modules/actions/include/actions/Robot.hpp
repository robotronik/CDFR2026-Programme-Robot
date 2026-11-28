#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "actions/ActuatorsControl.hpp"
#include "actions/SensorControl.hpp"
#include "defs/constante.h"
#include "defs/mainState.hpp"
#include "defs/structs.hpp"
#include "navigation/driveControl.h"

class Arduino;
class Navigation;
class TableState;
class VirtualStrategy;

/*
    Single entry point to the robot. Owns the hardware (Arduino, and the
    SensorControl bundle: lidar, camera, mat, buttons), the chassis
    (DriveControl), the actuators (ActuatorsControl) and the match-level state
    (team colour, calibration age, strategy pool). The TableState is owned by
    the app and only referenced here.
*/
class Robot {
    private:
        // Hardware is declared before the controllers that reference it so the
        // construction order in Robot.cpp is safe (each points at the previous).
        std::unique_ptr<Arduino> arduino_;
        SensorControl sensor_;

        // The table state is owned by the app and shared in via setTableState().
        TableState* tableState_ = nullptr;
        // Navigation is built once the table exists (see setTableState()).
        std::unique_ptr<Navigation> navigation_;
        MainState* currentState_ = nullptr;
        bool hardwareReady_ = false;

        // Pending requests from the API thread, drained by updateTeamStrat().
        std::mutex requestMutex_;
        bool hasColorRequest_ = false;
        ColorTeam requestedColor_ = ColorTeam::None;
        bool hasStrategyRequest_ = false;
        std::string requestedStrategy_;

        void initStrategies();

        // Per-loop sensor polling (lidar opponent detection, mat table feed).
        void pollLidar();
        void pollMat();
        // TODO: detect the opponent's action from its position.
        void opponentInAction(position_t position);

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
        Robot();
        ~Robot();

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

        // Match state (moved out of TableState).
        ColorTeam colorTeam = ColorTeam::None;
        int calibrationAge = 0;
        void resetCalibrationAge() { calibrationAge = 0; }

        // Strategy selection (moved out of the app globals).
        VirtualStrategy* currentStrategy = nullptr;
        std::vector<std::unique_ptr<VirtualStrategy>> strategies;

        // Name of the active strategy ("" when none).
        std::string strategyName() const;

        // Non-owning views over the strategy pool (used by the REST API).
        std::vector<std::string> strategyNames() const;
        bool hasStrategy(const std::string& name) const;

        // Table state (owned by the app, injected here).
        void setTableState(TableState* tableState);
        TableState& tableState() { return *tableState_; }
        Navigation& navigation() { return *navigation_; }

        // Wiring from the app.
        void setCurrentState(MainState* state) { currentState_ = state; }

        // Hardware accessors for the rest of the app.
        Arduino* arduino() const { return arduino_.get(); }
        SensorControl* sensor() { return &sensor_; }
        Lidar* lidar();
        Cam* cam();
        Mat* mat();

        // One-time bring-up (RGB, lidar setup, mat client, drive reset).
        // Returns false if a required device (the lidar) cannot be opened.
        bool startup();
        // Full teardown of the devices at process exit.
        void shutdown();
        // Per-loop sensor refresh: drive update, camera prior, lidar and mat.
        void update();

        // Bring the hardware up for a match; idempotent.
        void startMatchHardware();
        // Stop the match hardware (drive, actuators, lidar, camera).
        void stopMatchHardware();

        // Team / strategy switching.
        void switchTeamSide(ColorTeam color);
        void switchStrategy(const std::string& strategy);

        // Thread-safe request entry points (called from the API thread) and the
        // main-thread consumer that applies them.
        void requestColor(ColorTeam color);
        void requestStrategy(const std::string& strategy);
        void updateTeamStrat();
};
