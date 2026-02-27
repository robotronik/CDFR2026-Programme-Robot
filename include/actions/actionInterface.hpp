#pragma once
#include "defs/structs.hpp"
#include "defs/constante.h"
#include "defs/tableState.hpp"
#include "navigation/navigation.h"
#include "drive_interface.h"
#include "actions/strats.hpp"
#include "main.hpp"

class ActionInterface
{
    public:
        virtual ~ActionInterface() = default;

        // Consider using enum class for better type safety
        typedef enum
        {   
            FSM_RETURN_NONE = 0x0,
            FSM_RETURN_READY   = 0x1,
            FSM_RETURN_WORKING =0x2,
            FSM_RETURN_DONE    =0x3,
            FSM_RETURN_ERROR   =0x4
        } ReturnFSM_t;

        virtual ReturnFSM_t FSM_run() = 0;
        virtual position_t bestMove() = 0;
        virtual void reset() = 0;

        ReturnFSM_t getActionState(){
            return actionState;
        };

        void setActionState(ReturnFSM_t state){
            actionState = state;
        }

        void setActionID(int id){
            action_id = id;
        }

        int getActionID(){
            return action_id;
        }

        int* getActionIDPtr(){
            return &action_id;
        }
        
        void setNavRet(nav_return_t ret){
            nav_ret = ret;
        }

        nav_return_t getNavRet(){
            return nav_ret;
        }
    
    private:
        int action_id; // ID of the action, if only one action is possible will be 0, 
                       // if multiple actions possible will be set by the action manager as best action to do 
                       // (for example if multiple stocks are available, the action manager will set the action_id of the best stock to gather)
        ReturnFSM_t actionState = FSM_RETURN_NONE;
        nav_return_t nav_ret;

};
