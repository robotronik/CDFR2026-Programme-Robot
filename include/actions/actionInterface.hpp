#pragma once
#include "defs/structs.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h"
#include "actions/functions.h"

class ActionInterface
{
    public:
        virtual ~ActionInterface() = default;

        // Consider using enum class for better type safety
        typedef enum
        {   
            FSM_RETURN_NONE = 0x0,
            FSM_RETURN_WORKING =0x1,
            FSM_RETURN_DONE    =0x2,
            FSM_RETURN_ERROR   =0x3
        } ReturnFSM_t;

        virtual ReturnFSM_t FSM_run() = 0;
        virtual position_t bestMove() = 0;
        virtual void reset() = 0;

        ReturnFSM_t getActionState(){
            return actionState;
        };

        void setActionID(int id){
            action_id = id;
        }

        int getActionID(){
            return action_id;
        }
    
    private:
        int action_id; // ID of the action, if only one action is possible will be 0, 
                       // if multiple actions possible will be set by the action manager as best action to do 
                       // (for example if multiple stocks are available, the action manager will set the action_id of the best stock to gather)
        ReturnFSM_t actionState = FSM_RETURN_NONE;
        nav_return_t nav_ret;

};
