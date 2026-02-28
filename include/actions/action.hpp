#pragma once
#include "utils/logger.hpp"
#include "main.hpp"
#include "navigation/navigation.h"
#include "actions/functions.h"
#include "actions/strats.hpp"
#include "defs/tableState.hpp"
#include "defs/constante.h"

// Consider using enum class for better type safety
typedef enum
{
    FSM_RETURN_WORKING =0x0,
    FSM_RETURN_DONE    =0x1,
    FSM_RETURN_ERROR   =0x2
} ReturnFSM_t;

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

        ReturnFSM_t TakeStock();
        ReturnFSM_t DropStock();
        ReturnFSM_t Cursor();
        ReturnFSM_t GatherStock();
        ReturnFSM_t Calibrate();

        typedef enum
        {
            FSM_ACTION_GATHER,
            FSM_ACTION_DROP,
            FSM_ACTION_CURSOR,
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

        typedef enum
        {
            FSM_CURSOR_NAV,
            FSM_CURSOR_MOVE,
            FSM_CURSOR_END
        } StateCursor_t;

        StateGatherStock_t gatherStockState = FSM_GATHER_NAV;
        StateDropStock_t dropStockState = FSM_DROP_NONE;
        StateCursor_t CursorState = FSM_CURSOR_NAV;

        typedef enum
        {
            FSM_CALIBRATION_NAV,
            FSM_CALIBRATION_CALIBRATE
        } StateCalibration_t;

        StateCalibration_t calibrationState = FSM_CALIBRATION_NAV;
};