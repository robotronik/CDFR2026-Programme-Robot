#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "navigation/navigation.h" //For NavReturn & position_t

class RobotControl;

/*
    Action de calibration : force la calibration en se tournant vers
    un tag aruco (ou en s'en éloignant si trop proche/trop loin).
    Si la navigation échoue la calibration est considérée échouée
    (ReturnFSM::Error) et sera retentée une action plus tard.
*/
class CalibrationAction : public VirtualAction {
    public:
        CalibrationAction(RobotControl* robot, TableState* tableState);
        ~CalibrationAction() override = default;

        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float& reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        bool errorManagement(NavReturn error_code);
        bool successManagement();

    private:
        enum class StateCalibration {
            CalcCalculation,
            CalibrationNav,
        };

        RobotControl* robot;
        TableState* tableState;

        StateCalibration calibrationState;
        position_t calibrationTarget_;
        NavReturn nav_ret;

        // renvoie la position la plus proche
        // à adopter pour regarder un marqueur aruco et se recalibrer.
        position_t calculateClosestArucoPosition(position_t currentPos);
};