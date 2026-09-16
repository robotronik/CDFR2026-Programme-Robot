#pragma once
#include "drive_interface.h" // For position_t
#include "navigation/navigation.h" // For nav_return_t

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
        void SetBestAction();
        ReturnFSM_t Calibrate();
        nav_return_t nav_ret;
        
        /************  FSM GLOBAL ************/
        typedef enum
        {
            FSM_ACTION_NAV_HOME,
            FSM_ACTION_CALIBRATION,
            FSM_ACTION_WAIT
        } StateRun_t;
        StateRun_t runState = FSM_ACTION_CALIBRATION;

        /************  FSM CALIBRATION ************/
        typedef enum
        {   
            FSM_CALCULATION,
            FSM_CALIBRATION_NAV,
        } StateCalibration_t;
        StateCalibration_t calibrationState = FSM_CALCULATION;
};