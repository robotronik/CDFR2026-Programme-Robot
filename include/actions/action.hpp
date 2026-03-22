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
        
    private:
        /***** FUNCTIONS  *******/
        void SetBestAction(position_t position);

        ReturnFSM_t TakeStock();
        ReturnFSM_t StealStock();
        ReturnFSM_t DropStock();
        ReturnFSM_t Cursor();
        ReturnFSM_t Calibrate();
        ReturnFSM_t GetRobotCenter();

        int stock_num;// Num of stock
        int dropzone_num;// Num of dropzone to drop the stock or to steal from
        int offset;// Offset  is direction to take the stock from
        int steal_count; // Number of blocks taken from dropZone
        bool rotate_done = false;
        position_t backPos;

        nav_return_t nav_ret;
        position_t dropzonePos;
        position_t targetStockPos;
        position_t targetStockFirstPos;
        bool stockOrder[4];

        /************  FSM GLOBAL ************/
        typedef enum
        {
            FSM_ACTION_GATHER,
            FSM_ACTION_STEAL,
            FSM_ACTION_DROP,
            FSM_ACTION_CURSOR,
            FSM_ACTION_NAV_HOME,
            FSM_ACTION_CALIBRATION,
            FSM_CENTER_CALIBRATION
        } StateRun_t;
        StateRun_t runState = FSM_ACTION_GATHER;

        /************  FSM GATHER ************/
        typedef enum
        {
            FSM_GATHER_NAV,
            FSM_GATHER_DETECT,
            FSM_GATHER_CLAWS,
            FSM_GATHER_PREMOVE,
            FSM_GATHER_MOVE,
            FSM_GATHER_COLLECT,
            FSM_GATHER_COLLECTED
        } StateGatherStock_t;
        StateGatherStock_t gatherStockState = FSM_GATHER_NAV;
        
        /************ FSM STEAL **************/

        StateGatherStock_t stealStockState = FSM_GATHER_DETECT;

        /************  FSM DROP ************/
        typedef enum
        {
            FSM_DROP_NONE,
            FSM_DROP_NAV,
            FSM_DROP,
            FSM_DROP_NAV_BACK
        } StateDropStock_t;
        StateDropStock_t dropStockState = FSM_DROP_NONE;

        /************  FSM CURSOR ************/
        typedef enum
        {
            CURSOR_RAISE_CLAW,
            FSM_CURSOR_NAV,
            FSM_CURSOR_LOW_CLAW,
            FSM_CURSOR_MOVE,
            FSM_CURSOR_END
        } StateCursor_t;
        StateCursor_t CursorState = CURSOR_RAISE_CLAW;

        /************  FSM CALIBRATION ************/
        typedef enum
        {   
            FSM_CALCULATION,
            FSM_CALIBRATION_NAV,
        } StateCalibration_t;
        StateCalibration_t calibrationState = FSM_CALCULATION;

        /************  FSM CENTER CALIBRATION ************/
        typedef enum
        {
            FSM_ARUCO_1,
            FSM_ARUCO_2,
            FSM_ARUCO_NAV

        } StateCalibrationCamera_t;
        StateCalibrationCamera_t calibrationCameraState = FSM_ARUCO_1;


};