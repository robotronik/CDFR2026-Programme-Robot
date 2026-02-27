#pragma once
#include "utils/logger.hpp"
#include "main.hpp"
#include "navigation/navigation.h"
#include "actions/functions.h"
#include "actions/strats.hpp"
#include "defs/tableState.hpp"
#include "defs/constante.h"
#include "actions/gatherAction.hpp"
#include "actions/dropAction.hpp"

class ActionFSM{
    public:
        ActionFSM();
        ~ActionFSM();
        void Reset();
        bool RunFSM();
    private:

        GatherAction gatherAction; 
        DropAction dropAction;

        typedef enum
        {
            FSM_ACTION_GATHER,
            FSM_ACTION_DROP,
            FSM_ACTION_NAV_HOME
        } StateRun_t;

        StateRun_t runState = FSM_ACTION_NAV_HOME;
        StateRun_t getBestAction();
        
        ActionInterface::ReturnFSM_t Calibrate();

        typedef enum
        {
            FSM_CALIBRATION_NAV,
            FSM_CALIBRATION_CALIBRATE
        } StateCalibration_t;

        StateCalibration_t calibrationState = FSM_CALIBRATION_NAV;
};