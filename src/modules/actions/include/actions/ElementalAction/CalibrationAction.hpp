#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "navigation/navigation.h" //For NavReturn & position_t

class Robot;

/*
    Calibration action: forces a calibration by turning towards an aruco tag
    (or moving away from it if too close/far). If navigation fails, the
    calibration is considered failed (ReturnFSM::Error) and will be retried one
    action later.
*/
class CalibrationAction : public VirtualAction {
    public:
        CalibrationAction(Robot* robot, TableState* tableState);
        ~CalibrationAction() override = default;

        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float& reward) override;
        bool fullBlock() override;
        bool movementBlock() override;

    protected:
        bool errorManagement(NavReturn error_code);
        bool successManagement();

    private:
        enum class StateCalibration {
            CalcCalculation,
            CalibrationNav,
        };

        Robot* robot;
        TableState* tableState;

        StateCalibration calibrationState;
        position_t calibrationTarget_;
        NavReturn nav_ret;

        // Returns the nearest position to adopt in order to look at an aruco
        // marker and recalibrate.
        position_t calculateClosestArucoPosition(position_t currentPos);
};