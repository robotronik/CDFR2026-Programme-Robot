#include <string>
#include <exception>
#include "actions/action.hpp"
#include "utils/logger.hpp"
#include "main.hpp"
#include "navigation/navigation.h"
#include "actions/functions.h"
#include "actions/strats.hpp"
#include "defs/tableState.hpp"
#include "defs/constante.h"

ActionFSM::ActionFSM(){
    Reset();
}

ActionFSM::~ActionFSM(){}

void ActionFSM::Reset(){
    runState = FSM_ACTION_GATHER;
    gatherStockState = FSM_GATHER_NAV;
}

bool ActionFSM::RunFSM(){
    ReturnFSM_t ret;
    switch (runState)
    {
    //****************************************************************
    case FSM_ACTION_GATHER:
        ret = GatherStock();
        if (ret == FSM_RETURN_DONE)
            runState = FSM_ACTION_NAV_HOME;
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("Couldn't gather");
            // TODO Handle error
        }
        break;
    //****************************************************************
    case FSM_ACTION_NAV_HOME:
        if (returnToHome()){
            runState = FSM_ACTION_GATHER;
            return true; // Robot is done
        }
        break;
    }
    return false;
}


ReturnFSM_t ActionFSM::GatherStock(){
    nav_return_t nav_ret;
    switch (gatherStockState){
    case FSM_GATHER_NAV:
        // TODO Highways should be enabled for some takes
        position_t pos;
        nav_ret = navigationGoTo(pos, Direction::SHORTEST, Rotation::SHORTEST, Rotation::SHORTEST);
        if (nav_ret == NAV_DONE){
            gatherStockState = FSM_GATHER_COLLECT;
            LOG_INFO("Nav done and RevolverPrepareLowBarrel done for FSM_GATHER_NAV, going to FSM_GATHER_MOVE");
        }
        else if (nav_ret == NAV_ERROR){
            // TODO get another stock
            return FSM_RETURN_ERROR;
        }
        break;
    case FSM_GATHER_COLLECT:
        // Collect the stock
        if (true){ //takeStockPlatforms()
            gatherStockState = FSM_GATHER_NAV;
            LOG_INFO("taking stock for FSM_GATHER_COLLECT");
            return FSM_RETURN_DONE;
        }
        break;
    }
    return FSM_RETURN_WORKING;
}