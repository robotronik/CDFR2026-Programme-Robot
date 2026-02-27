#include "actions/gatherAction.hpp"

GatherAction::GatherAction(){
    reset();
}

void GatherAction::reset(){
    setActionID(-1);
    setActionState(FSM_RETURN_READY);
    gatherStockState = FSM_GATHER_NAV;
    offset = 0;
    stockOff = STOCK_OFFSETS[offset];
    stockPos = {0,0,0};
    targetPos = {0,0,0};
}

ActionInterface::ReturnFSM_t GatherAction::FSM_run(){
    if(getActionID()== -1){
        LOG_ERROR("Trying to run gather action while not ready");
        return FSM_RETURN_ERROR;
    }
    /*
    if (getActionID() == -1 && getGatherStockState() == FSM_GATHER_NAV){
        bestMove(); // Set the best move to do for the current stock
        if (getActionID() == -1){ // No stock to take
            LOG_INFO("No stock to gather, exiting FSM_run");
            return FSM_RETURN_DONE;
        }
    }*/

    position_t stockPos = STOCK_POSITIONS_TABLE[getActionID()];
    position_t stockOff = STOCK_OFFSETS[offset];
    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            LOG_INFO("Navigating to stock ", getActionID(), " at position (", targetPos.x, ",", targetPos.y, ") with angle ", targetPos.a);
            setNavRet(navigationGoTo(targetPos, true));
            if (getNavRet() == NAV_DONE){
                if (lowerClaws()){
                    gatherStockState = FSM_GATHER_MOVE;
                    LOG_INFO("Nav done FSM_GATHER_NAV, going to FSM_GATHER_MOVE");
                }
            }
            else if (getNavRet() == NAV_ERROR){
                gatherStockState = FSM_GATHER_NAV;
                LOG_WARNING("Navigation error while going to stock ", getActionID());
                // TODO get another stock
                return FSM_RETURN_ERROR;
            }
            }
            break;

        case FSM_GATHER_MOVE:
            setNavRet(navigationGoTo(position_t {stockPos.x + int(stockOff.x * 0.66), stockPos.y + int(stockOff.y * 0.66), angle}, true));
            //LOG_INFO("Moving to stock ", stock_num, " at position (", stockPos.x + int(stockOff.x * 0.7), ",", stockPos.y + int(stockOff.y * 0.7), ") with angle ", angle);
            if (getNavRet() == NAV_DONE){
                gatherStockState = FSM_GATHER_COLLECT;
                LOG_INFO("Nav done FSM_GATHER_MOVE, going to FSM_GATHER_COLLECT");
            }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the stock
            if (rotateTwoBlocks(false)){ // TODO fermer claw puis partir sans attendre fin rotateTwoBlocks (timer)
                LOG_INFO("Stock", getActionID(), " collected");
                setStockAsRemoved(getActionID());
                setActionState(FSM_RETURN_DONE);
                return FSM_RETURN_DONE;
            }
            break;
    }
    return FSM_RETURN_WORKING;
}

int GatherAction::chooseNextStock(){
    // Returns the number of the closest available stock to be taken
    double min = INFINITY;
    int closest_stock = -1;
    for (int i = 0; i < STOCK_COUNT; i++){
        if (tableStatus.avail_stocks[i]){
            double dist = position_distance(drive.position, STOCK_POSITIONS_TABLE[i]);
            if (dist < min){
                min = dist;
                closest_stock = i;
            }
        }
    }
    if (closest_stock == -1){
        LOG_GREEN_INFO("No next stock available");
        return -1;
    }else{
        LOG_INFO("Next stock to take: ", closest_stock);
        return closest_stock;
    }
}

bool GatherAction::chooseStockStrategy(int& stockNum, int& stockOffset){
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);

    int todo_stocks[9];
    int num = 0;
    bool endlessMode = false; // If true, the robot will take all the stocks in order, ignoring the strategy (for testing)
    switch (strategy)
    {   
        case 1:
            todo_stocks[0] = 0;
            num = 1;
            break;
        case 2:
            todo_stocks[0] = 7;
            todo_stocks[1] = 6;
            num = 2;
            break;
        case 3:
            todo_stocks[0] = 3;
            num = 1;
            break;
        case 4:
            todo_stocks[0] = 0;
            num = 1;
            endlessMode = true;
            break;
    }

    if (color == YELLOW){
        for (int i = 0; i < num; i++){
            todo_stocks[i] = (todo_stocks[i] + STOCK_COUNT/2) % STOCK_COUNT;
        }
    }
    int i = 0;
    while (i < num){
        if (tableStatus.avail_stocks[todo_stocks[i]]){
            stockNum = todo_stocks[i];
            stockOffset = getBestStockPositionOff(stockNum, drive.position);
            return true;
        }
        i++;
    }

    if (endlessMode){
        stockNum = (stockNum + 1) % STOCK_COUNT; // In endless mode, we take the stocks in order
        stockOffset = getBestStockPositionOff(stockNum, drive.position);
        return true;
    }

    int nextStock = chooseNextStock(); // Choose the closest stock if the strategy stocks are not available
    if (nextStock != -1){
        stockNum = nextStock;
        stockOffset = getBestStockPositionOff(stockNum, drive.position);
        return true;
    }
    LOG_GREEN_INFO("No stock available");
    return false;
}

int GatherAction::getBestStockPositionOff(int stockNum, position_t fromPos){
    int bestOff = -1;
    double bestDist2 = INFINITY;

    position_t stockPos = STOCK_POSITIONS_TABLE[stockNum];

    for (int i = 0; i < 2; i++){
        int offNum = STOCK_OFFSET_MAPPING[stockNum][i];
        if (offNum == -1)
            continue;

        position_t stockOff = STOCK_OFFSETS[offNum];
        position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, 0};

        double dist2 = position_distance(stockOff,targetPos);

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestOff = offNum;
        }
    }

    return bestOff;
}

void GatherAction::setStockAsRemoved(int num){
    tableStatus.avail_stocks[num] = false;
    LOG_INFO("Removed stock ", num);
}

position_t GatherAction::bestMove(){
    if(getActionState() != FSM_RETURN_READY){
        LOG_WARNING("Trying to get best move for gather action while stock already collected, returning pos null");
        return {0,0,0};
    }
    if(chooseStockStrategy(*getActionIDPtr(), offset)){
        stockPos = STOCK_POSITIONS_TABLE[getActionID()];
        stockOff = STOCK_OFFSETS[STOCK_OFFSET_MAPPING[getActionID()][offset]];
        targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a};
        double angle = RAD_TO_DEG*  position_angle(targetPos, stockPos);
        LOG_DEBUG("Set best move for stock ", getActionID(), " is position (", targetPos.x, ",", targetPos.y, ") with angle ", angle);
        return position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
    }else{
        LOG_INFO("No more stocks to take, exiting GatherStock");
        setActionID(-1);
        return position_t{0,0,0};
    }
    
}