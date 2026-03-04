#pragma once

#include "drive_interface.h" // For position_t
#include "navigation/navigation.h" // For nav_return_t

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
        void SetBestAction(position_t position);
        inline bool cursorIsDone(){ return cursorStatus; }
        inline void setCursorIsDone(bool val){ cursorStatus = val; }
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
        ReturnFSM_t GetRobotCenter();
        ReturnFSM_t Calibrate();

        typedef enum
        {
            FSM_ACTION_GATHER,
            FSM_ACTION_DROP,
            FSM_ACTION_CURSOR,
            FSM_ACTION_NAV_HOME,
            FSM_CENTER_CALIBRATION
        } StateRun_t;

        StateRun_t runState = FSM_ACTION_GATHER;

        typedef enum
        {
            FSM_GATHER_NAV,
            FSM_GATHER_CLAWS,
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
        bool cursorStatus = false;

        typedef enum
        {
            FSM_ARUCO_1,
            FSM_ARUCO_2,
            FSM_ARUCO_NAV

        } StateCalibrationCamera_t;

        typedef enum
        {
            FSM_CALIBRATION_NAV,
            FSM_CALIBRATION_CALIBRATE,
            FSM_CALCULATION
        } StateCalibration_t;

        StateCalibration_t calibrationState = FSM_CALCULATION;

        StateCalibrationCamera_t calibrationCameraState = FSM_ARUCO_1;

};