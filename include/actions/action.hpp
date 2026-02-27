#pragma once
#include "utils/logger.hpp"
#include "main.hpp"
#include "navigation/navigation.h"
#include "actions/functions.h"
#include "actions/strats.hpp"
#include "defs/tableState.hpp"
#include "defs/constante.h"
#include "actions/actionInterface.hpp"

class ActionFSM{
    public:
        ActionFSM();
        ~ActionFSM();
        void Reset();
        bool RunFSM();

    private:

        int stock_num;// Num of stock
        int dropzone_num;// Num of dropzone to drop the stock
        int offset;// Offset  is direction to take the stock from
        nav_return_t nav_ret;
        position_t dropzonePos;

        ActionInterface::ReturnFSM_t TakeStock();
        ActionInterface::ReturnFSM_t DropStock();
        ActionInterface::ReturnFSM_t GatherStock();
        ActionInterface::ReturnFSM_t Calibrate();

        typedef enum
        {
            FSM_ACTION_GATHER,
            FSM_ACTION_DROP,
            FSM_ACTION_NAV_HOME
        } StateRun_t;

        StateRun_t runState = FSM_ACTION_NAV_HOME;

        typedef enum
        {
            FSM_GATHER_NAV,
            FSM_GATHER_MOVE,
            FSM_GATHER_COLLECT,
            FSM_GATHER_COLLECTED
        } StateGatherStock_t;

        typedef enum
        {
            FSM_DROP_NONE,
            FSM_DROP_NAV,
            FSM_DROP
        } StateDropStock_t;

        StateGatherStock_t gatherStockState = FSM_GATHER_NAV;
        StateDropStock_t dropStockState = FSM_DROP_NONE;

        typedef enum
        {
            FSM_CALIBRATION_NAV,
            FSM_CALIBRATION_CALIBRATE
        } StateCalibration_t;

        StateCalibration_t calibrationState = FSM_CALIBRATION_NAV;
};