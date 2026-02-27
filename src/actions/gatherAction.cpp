#include "actions/gatherAction.hpp"

GatherAction::GatherAction(){

}

ActionInterface::ReturnFSM_t GatherAction::FSM_run(){
    if (getActionID() == -1 || gatherStockState == FSM_GATHER_NAV){
        LOG_DEBUG("Getting next stock to take");
        if (!chooseStockStrategy(getActionID(), offset)){
            LOG_INFO("No more stocks to take, exiting GatherStock");
            setActionID(-1);
            gatherStockState = FSM_GATHER_NAV;
            return FSM_RETURN_DONE;
        }
        LOG_INFO("Next stock to take: ", getActionID(), " offset: ", offset);
    }

    position_t stockPos = STOCK_POSITIONS_TABLE[getActionID()];
    position_t stockOff = STOCK_OFFSETS[STOCK_OFFSET_MAPPING[getActionID()][offset]];
    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
            LOG_INFO("Navigating to stock ", getActionID(), " at position (", targetPos.x, ",", targetPos.y, ") with angle ", targetPos.a);

            nav_ret = navigationGoTo(targetPos, true);
            if (nav_ret == NAV_DONE){
                if (lowerClaws()){
                    gatherStockState = FSM_GATHER_MOVE;
                    LOG_INFO("Nav done FSM_GATHER_NAV, going to FSM_GATHER_MOVE");
                }
            }
            else if (nav_ret == NAV_ERROR){
                gatherStockState = FSM_GATHER_NAV;
                LOG_WARNING("Navigation error while going to stock ", getActionID());
                // TODO get another stock
                return FSM_RETURN_ERROR;
            }
            }
            break;

        case FSM_GATHER_MOVE:
            nav_ret = navigationGoTo(position_t {stockPos.x + int(stockOff.x * 0.9), stockPos.y + int(stockOff.y * 0.9), angle}, true);
            if (nav_ret == NAV_DONE){
                gatherStockState = FSM_GATHER_COLLECT;
                LOG_INFO("Nav done FSM_GATHER_MOVE, going to FSM_GATHER_COLLECT");
            }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the stock
            if (rotateTwoBlocks(false)){ // TODO fermer claw puis partir sans attendre fin rotateTwoBlocks (timer)
                LOG_INFO("Stock", getActionID(), " collected");
                setStockAsRemoved(getActionID());
                gatherStockState = FSM_GATHER_COLLECTED;
                return FSM_RETURN_DONE;
            }
            break;
        case FSM_GATHER_COLLECTED:
            // Wait for the stock to be collected before doing anything else (like navigating to dropzone), to avoid dropping the stock on the way
            LOG_INFO("GATHER_COLLECTED");
            return FSM_RETURN_DONE;
    }
    return FSM_RETURN_WORKING;
}
