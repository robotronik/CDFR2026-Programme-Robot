#include "actions/dropAction.hpp"

DropAction::DropAction(){
    reset();
}

void DropAction::reset(){
    dropState = FSM_DROP_NAV;
    dropzonePos = {0,0,0};
    targetPos = {0,0,0};
    setActionID(-1);
    setNavRet(NAV_IN_PROCESS);
}

int DropAction::GetBestDropZone(position_t fromPos){
    int bestDropZone = -1;
    double bestDist2 = 1000000;

    for (int i = 0; i < DROPZONE_COUNT; i++){
        if (tableStatus.dropzone_states[i] != TableState::DROPZONE_EMPTY)
            continue;

        position_t dropzonePos = DROPZONE_POSITIONS_TABLE[i];
        double dist2 = position_distance(fromPos, dropzonePos) + abs( tableStatus.colorTeam == BLUE ? dropzonePos.y - 1500 : dropzonePos.y + 1500); // We want to favor the dropzones on our side of the table

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestDropZone = i;
        }
    }

    return bestDropZone;
}

position_t DropAction::bestMove(){
    if(getActionState() != FSM_RETURN_READY){
        LOG_WARNING("Trying to get best move for drop action while not ready, returning pos null");
        return {0,0,0};
    }
    setActionID(GetBestDropZone(drive.position));
    LOG_DEBUG("best drop zone for stock is ", getActionID());
    dropzonePos = DROPZONE_POSITIONS_TABLE[getActionID()];
    targetPos = getBestDropZonePosition(getActionID(), drive.position);
    LOG_DEBUG("Dropzone position for stock is (", targetPos.x, ",", targetPos.y, targetPos.a, ")");
    return targetPos;
}

ActionInterface::ReturnFSM_t DropAction::FSM_run(){
    if(getActionID() == -1){
        LOG_ERROR("Trying to run drop action while not ready");
        return FSM_RETURN_ERROR;
    }

    switch (dropState){
        case FSM_DROP_NAV:
            {   
            // Navigate to dropzone

            setNavRet(navigationGoTo(targetPos, true));
            if (getNavRet() == NAV_DONE){ // We consider that we are at the dropzone if we are close enough, to avoid navigation errors
                LOG_INFO("Nav done FSM_DROP_NAV, going to FSM_DROP");
                setDropzoneState(getActionID(), (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_YELLOW : TableState::DROPZONE_BLUE); // Mark dropzone as occupied
                dropState = FSM_DROP_DROP;
            }
            else if (getNavRet() == NAV_ERROR){

                LOG_WARNING("Navigation error while going to dropzone ", getActionID());
                setDropzoneAsError(getActionID());
                
                int dropzone_temp = GetBestDropZone(drive.position);
                if(dropzone_temp == -1){
                    LOG_ERROR("No more dropzone available, cannot drop stock ");
                    return ActionInterface::FSM_RETURN_ERROR;
                }else{
                    setDropzoneState(getActionID(), TableState::DROPZONE_EMPTY); // Reset previous dropzone state
                    setActionID(dropzone_temp);
                    targetPos = getBestDropZonePosition(getActionID(), drive.position);
                    LOG_INFO("Dropzone position for stock is (", targetPos.x, ",", targetPos.y, ")");
                }

                dropState = FSM_DROP_NAV;
                return ActionInterface::FSM_RETURN_ERROR;
            }
            }
            break;

        case FSM_DROP_DROP:
            // Drop the stock
            if (dropBlock()){
                LOG_INFO("Stock dropped at dropzone ", getActionID());
                setActionState(FSM_RETURN_DONE);
                return ActionInterface::FSM_RETURN_DONE; 
            }
            break;
    }
    return ActionInterface::FSM_RETURN_WORKING;
}

position_t DropAction::getBestDropZonePosition(int dropzoneNum, position_t fromPos){
    position_t dropzonePos = DROPZONE_POSITIONS_TABLE[dropzoneNum];
    position_t vect = position_vector(dropzonePos, fromPos);
    position_normalize(vect);
    position_t bestPoss = position_t{dropzonePos.x + int(vect.x * OFFSET_DROPZONE), dropzonePos.y + int(vect.y * OFFSET_DROPZONE), RAD_TO_DEG * position_angle(fromPos, dropzonePos)};
    return bestPoss;
}

void DropAction::setDropzoneState(int dropzoneNum, TableState::dropzone_state_t state){
    tableStatus.dropzone_states[dropzoneNum] = state;
    LOG_INFO("Set dropzone ", dropzoneNum, " state to ", state);
}

void DropAction::setDropzoneAsError(int dropzoneNum){
    setDropzoneState(dropzoneNum, TableState::DROPZONE_ERROR);
}