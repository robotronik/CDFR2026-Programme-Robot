#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h"
#include "navigation/navigation.h" //For nav_return_t & position_t

/*
    Action de calibration : force la calibration en se tournant vers
    un tag aruco (ou en s'en éloignant si trop proche/trop loin).
    Si la navigation échoue la calibration est considérée échouée
    (FSM_RETURN_ERROR) et sera retentée une action plus tard.
*/
class CalibrationAction : public VirtualAction {
    public:
        CalibrationAction(DriveControl* drive, TableState* tableState);
        ~CalibrationAction() override = default;

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        float available() override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        bool errorManagement(nav_return_t error_code);
        bool successManagement();

    private:
        typedef enum
        {
            FSM_CALCULATION,
            FSM_CALIBRATION_NAV,
        } StateCalibration_t;

        DriveControl* drive;
        TableState* tableState;

        StateCalibration_t calibrationState;
        position_t calibrationTarget_;
        nav_return_t nav_ret;

        // Anciennement dans strats.cpp : renvoie la position la plus proche
        // à adopter pour regarder un marqueur aruco et se recalibrer.
        position_t calculateClosestArucoPosition(position_t currentPos);
};