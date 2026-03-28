#include "actions/strats.hpp"
#include "actions/functions.h"
#include "utils/logger.hpp"
#include "defs/structs.hpp"
#include "navigation/driveControl.h"
#include <math.h>
#include "main.hpp" // for tableStatus
#include "navigation/pathfind.h"

void check(colorTeam_t color, int strategy){
    // Check if the color and strategy are valid
    if (color == NONE || strategy < 1 || strategy > 4)
        LOG_ERROR("Invalid color (", color, ") or strategy (", strategy, ")");
}

// Function to handle the strategy
position_t StratStartingPos(){
    // Returns the starting position of the robot
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);
    position_t pos = {-725, 1125, 0};

    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}

int chooseNextStock(int& closest_stock, int& closest_offset){
    // Returns the number of the closest available stock to be taken
    int min = std::numeric_limits<int>::max();
    closest_stock = -1;
    closest_offset = -1;
    for (int i = 0; i < STOCK_COUNT; i++){
        if (tableStatus.avail_stocks[i]){            
            for (int j = 0; j < 2; j++){
                int offNum = STOCK_OFFSET_MAPPING[i][j];
                if (offNum == -1)
                    continue;

                int dist2 = toAStarDistStock(i, offNum);

                if (dist2 < min){
                    min = dist2;
                    closest_stock = i;
                    closest_offset = offNum;
                }
            }
        }
    }
    if (closest_stock == -1){
        LOG_WARNING("No next stock available");
        return -1;
    }else{
        //LOG_INFO("Next stock to take: ", closest_stock);
        return closest_stock;
    }
}

int toAStarDistStock(int stockNum, int stockOffset){
    position_t stockPos = STOCK_POSITIONS_TABLE[stockNum];
    position_t stockOff = STOCK_OFFSETS[stockOffset];
    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);
    position_t target = position_sum(stockPos, stockOff);
    target.a = angle;
    return toAStarDist(target);
}

int toAStarDist(position_t a){
    int lenght;
    position_t path[1024];
    pathfind(drive.position, a, path, &lenght);
    return lenght;
}

int chooseStockStrategy(int& stockNum, int& stockOffset){
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);

    int todo_stocks[9];
    int num = 0;
    bool endlessMode = false; // If true, the robot will take all the stocks in order, ignoring the strategy (for testing)
    LOG_EXTENDED_DEBUG("Strategy", strategy);
    switch (strategy)
    {   
        case 1:
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
            int dist = getBestStockPositionOff(stockNum, stockOffset);
            if(!dist){
                return std::numeric_limits<int>::max();
            }
            return dist;
        }
        i++;
    }

    if (endlessMode){
        stockNum = (stockNum + 1) % STOCK_COUNT; // In endless mode, we take the stocks in order
        int dist = getBestStockPositionOff(stockNum, stockOffset);
        
        if(!dist){
                return std::numeric_limits<int>::max();
            }
            return dist;
    }

    int dist = chooseNextStock(stockNum, stockOffset); // Choose the closest stock if the strategy stocks are not available
    if (stockNum != -1){
        return dist;
    }
    //LOG_WARNING("No stock available");
    return 0;
}

// Return the closest position to look at an aruco marker
position_t calculateClosestArucoPosition(position_t currentPos){
    position_t outPos = currentPos;
    position_t closestPos = ARUCO_POSITIONS_TABLE[0];
    double minDistance = position_distance(currentPos, ARUCO_POSITIONS_TABLE[0]);
    for (int i = 1; i < 4; i++){
        double d = position_distance(currentPos, ARUCO_POSITIONS_TABLE[i]);
        if (d < minDistance){
            minDistance = d;
            closestPos = ARUCO_POSITIONS_TABLE[i];
        }
    }
    LOG_ERROR("Distance to closest aruco marker: ", minDistance);
    const double target_distance_min = 300.0; // mm
    const double target_distance_max = 650.0; // mm

    if (minDistance < target_distance_min || minDistance > target_distance_max){
        LOG_WARNING("Not in valid range, moving to preset position");
        // Calculate the closest valid position using predetermined pos
        outPos = ARUCO_CALIB_POSITIONS[0];
        double minTargetDistance = 1e6;
        for (int i = 0; i < ARUCO_CALIB_POSITIONS_COUNT; i++){
            // Blue side
            double d = position_distance(currentPos, ARUCO_CALIB_POSITIONS[i]);
            if (d < minTargetDistance){
                minTargetDistance = d;
                outPos = ARUCO_CALIB_POSITIONS[i];
            }
            // Yellow side (mirrored)
            position_t mirroredPos = ARUCO_CALIB_POSITIONS[i];
            mirroredPos.y = -mirroredPos.y;
            d = position_distance(currentPos, mirroredPos);
            if (d < minTargetDistance){
                minTargetDistance = d;
                outPos = mirroredPos;
            }
        }
    }
    else {
        LOG_DEBUG("Good distance → no movement");
    }
    outPos.a = RAD_TO_DEG * position_angle(outPos, closestPos) + OFFSET_CAM_A;

    return outPos;
}

/*
    Return best drop zone id
    -1 if no drop zone available
*/
int GetBestDropZone(){
    int bestDropZone = -1;
    int bestDist2 = std::numeric_limits<int>::max();

    for (int i = 0; i < DROPZONE_COUNT; i++){
        if (tableStatus.dropzone_states[i] != TableState::DROPZONE_EMPTY)
            continue;

        position_t dropzonePos = DROPZONE_POSITIONS_TABLE[i];
        int dist2 = toAStarDist(dropzonePos);

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestDropZone = i;
        }
    }

    return bestDropZone;
}

int getBestStockPositionOff(int& stockNum, int& bestDist){
    int bestOff = -1;
    int bestDist2 = std::numeric_limits<int>::max();

    if(STOCK_OFFSET_MAPPING[stockNum][1] == -1){
        return 0;
    }

    for (int i = 0; i < 2; i++){
        int offNum = STOCK_OFFSET_MAPPING[stockNum][i];
        if (offNum == -1)
            continue;

        int dist2 = toAStarDistStock(stockNum, offNum);

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestOff = offNum;
        }
    }
    bestDist = bestDist2;
    return bestOff;
}

int getBestDropZonePosition(int dropzoneNum, position_t& bestPoss, bool steal){
    double dropZoneOffset = OFFSET_DROPZONE;
    TableState::dropzone_state_t zone_of_interest = TableState::DROPZONE_EMPTY;
    if(steal){
        dropZoneOffset = OFFSET_STOCK*1.2;
        zone_of_interest = (tableStatus.colorTeam == YELLOW ? TableState::DROPZONE_BLUE : TableState::DROPZONE_YELLOW);
    }

    int min = std::numeric_limits<int>::max();
    dropzoneNum = -1;

    int d1;
    position_t temp_pos;

    for(int k = 0; k< DROPZONE_COUNT; k++){
        if(tableStatus.dropzone_states[k] != zone_of_interest) continue;
        if (k == 7 || k == 4 || k == 2 ){
            temp_pos = DROPZONE_POSITIONS_TABLE[k];
            /*deactivated for now
            position_t vect = position_vector(dropzonePos, fromPos);
            position_normalize(vect);
            position_t bestPoss = position_t{dropzonePos.x + int(vect.x * OFFSET_DROPZONE), dropzonePos.y + int(vect.y * OFFSET_DROPZONE), RAD_TO_DEG * position_angle(fromPos, dropzonePos)};
            */
           d1 = toAStarDist(position_sum(temp_pos, position_t{.x = dropZoneOffset, .y= -OFFSET_CLAW_Y/2}));
           int d2 = toAStarDist(position_sum(temp_pos, position_t{.x = -1 * dropZoneOffset, .y= OFFSET_CLAW_Y/2}));
            if(d1 < d2 ){
                    temp_pos = position_sum(temp_pos, position_t{.x = dropZoneOffset, .y= -OFFSET_CLAW_Y/2});
                    temp_pos.a = 180;
            }else{
                temp_pos = position_sum(temp_pos, position_t{.x =  -1 * dropZoneOffset, .y= OFFSET_CLAW_Y/2});
                temp_pos.a = 0;
                d1 = d2;
            }
            if(min > d1){
                bestPoss = temp_pos;
                min = d1;
            }
        }else{
            temp_pos = DROPZONE_POSITIONS_TABLE[dropzoneNum];
            if(MAX_WIDTH_TABLE - abs(temp_pos.x) < MAX_LENGTH_TABLE - abs(temp_pos.y)){
                temp_pos.x += (temp_pos.x > 0? -1 : 1 ) * dropZoneOffset;
                temp_pos.y += (temp_pos.x > 0? 1 : -1) * OFFSET_CLAW_Y/2;
                temp_pos.a = (temp_pos.x > 0? 0 : 180);
            }else{
                temp_pos.y += (temp_pos.y > 0? -1 : 1 ) * dropZoneOffset;
                temp_pos.x += (temp_pos.y > 0? -1 : 1 ) * OFFSET_CLAW_Y/2;
                temp_pos.a = (temp_pos.y > 0? 90 : -90);
            }
            d1 = toAStarDist(temp_pos);
            
        }
        if(min > d1){
            bestPoss = temp_pos;
            min = d1;
        }
    }
    return (dropzoneNum != -1 ? min : 0);
}

/*
    Determine the best drop zone from wich to steal
    For now very simple 
    TODO: developped with adversary position
*/
int getBestStealZonePosition(int& bestDropZone, position_t& bestPos){
    return getBestDropZonePosition(bestDropZone, bestPos, true);
}

position_t getBestIsolatedPosition(position_t centerPos, position_t fromPos){
    const float recul = STOCKS_LENGTH / 2 + ROBOT_WIDTH / 2;
    position_t vect = position_t{ .x  = sin(DEG_TO_RAD * (centerPos.a + 90)) * recul, .y = cos(DEG_TO_RAD * (centerPos.a + 90)) * recul, .a = 0};
    position_t possTarget1 = position_sum(centerPos, vect);

    vect = position_t{ .x  = sin(DEG_TO_RAD * (centerPos.a - 90)) * recul, .y = cos(DEG_TO_RAD * (centerPos.a - 90)) * recul, .a = 0};
    position_t possTarget2 = position_sum(centerPos, vect);

    if(position_distance(fromPos, possTarget1) < position_distance(fromPos, possTarget2)){ //TODO replace with A*
        return possTarget1;
    }
    return possTarget2;
}


