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

int chooseNextStock(){
    // Returns the number of the closest available stock to be taken
    double min = INFINITY;
    int closest_stock = -1;
    for (int i = 0; i < STOCK_COUNT; i++){
        if (tableStatus.avail_stocks[i]){
            double dist = toAStarDist(STOCK_POSITIONS_TABLE[i]);
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
    switch (strategy)
    {   
        case 1:
            break;
        case 2:
            todo_stocks[0] = 5;
            num = 1;
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
            return toAStarDistStock(stockNum, stockOffset);
        }
        i++;
    }

    if (endlessMode){
        stockNum = (stockNum + 1) % STOCK_COUNT; // In endless mode, we take the stocks in order
        stockOffset = getBestStockPositionOff(stockNum, drive.position);
        
        return toAStarDistStock(stockNum, stockOffset);
    }

    int nextStock = chooseNextStock(); // Choose the closest stock if the strategy stocks are not available
    if (nextStock != -1){
        stockNum = nextStock;
        stockOffset = getBestStockPositionOff(stockNum, drive.position);
        return toAStarDistStock(stockNum, stockOffset);
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
int GetBestDropZone(position_t fromPos){
    int bestDropZone = -1;
    double bestDist2 = INFINITY;

    for (int i = 0; i < DROPZONE_COUNT; i++){
        if (tableStatus.dropzone_states[i] != TableState::DROPZONE_EMPTY)
            continue;

        position_t dropzonePos = DROPZONE_POSITIONS_TABLE[i];
        double dist2 = toAStarDist(dropzonePos);

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestDropZone = i;
        }
    }

    return bestDropZone;
}

int getBestStockPositionOff(int stockNum, position_t fromPos){
    int bestOff = -1;
    int bestDist2 = std::numeric_limits<int>::max();

    position_t stockPos = STOCK_POSITIONS_TABLE[stockNum];

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

    return bestOff;
}

position_t getBestDropZonePosition(int dropzoneNum, bool steal){
    double dropZoneOffset = OFFSET_DROPZONE;
    if(steal) dropZoneOffset = OFFSET_STOCK;

    if (dropzoneNum == 7 || dropzoneNum == 4 || dropzoneNum == 2 ){
        position_t bestPoss = DROPZONE_POSITIONS_TABLE[dropzoneNum];
        /*deactivated for now
        position_t vect = position_vector(dropzonePos, fromPos);
        position_normalize(vect);
        position_t bestPoss = position_t{dropzonePos.x + int(vect.x * OFFSET_DROPZONE), dropzonePos.y + int(vect.y * OFFSET_DROPZONE), RAD_TO_DEG * position_angle(fromPos, dropzonePos)};
        */
        if(toAStarDist(position_sum(bestPoss, position_t{.x = dropZoneOffset, .y= -OFFSET_CLAW_Y/2}))
            < toAStarDist(position_sum(bestPoss, position_t{.x = -1 * dropZoneOffset, .y= OFFSET_CLAW_Y/2}))){
                bestPoss = position_sum(bestPoss, position_t{.x = dropZoneOffset, .y= -OFFSET_CLAW_Y/2});
                bestPoss.a = 180;
        }else{
            bestPoss = position_sum(bestPoss, position_t{.x =  -1 * dropZoneOffset, .y= OFFSET_CLAW_Y/2});
            bestPoss.a = 0;
        }
        return bestPoss;
    }else{
        position_t bestPoss = DROPZONE_POSITIONS_TABLE[dropzoneNum];
        if(MAX_WIDTH_TABLE - abs(bestPoss.x) < MAX_LENGTH_TABLE - abs(bestPoss.y)){
            bestPoss.x += (bestPoss.x > 0? -1 : 1 ) * dropZoneOffset;
            bestPoss.y += (bestPoss.x > 0? 1 : -1) * OFFSET_CLAW_Y/2;
            bestPoss.a = (bestPoss.x > 0? 0 : 180);
        }else{
            bestPoss.y += (bestPoss.y > 0? -1 : 1 ) * dropZoneOffset;
            bestPoss.x += (bestPoss.y > 0? -1 : 1 ) * OFFSET_CLAW_Y/2;
            bestPoss.a = (bestPoss.y > 0? 90 : -90);
        }
        return bestPoss;
    }
    
}

/*
    Determine the best drop zone from wich to steal
    For now very simple 
    TODO: developped with adversary position
*/
int getBestStealZonePosition(position_t fromPos, int& bestDropZone, position_t& bestPos){
    int min_distance = INFINITY;
    for(int idx = 0; idx < DROPZONE_COUNT; idx++){
        if(tableStatus.dropzone_states[idx] == (tableStatus.colorTeam == BLUE ? TableState::DROPZONE_YELLOW : TableState::DROPZONE_BLUE)){
            position_t tmp_pos = DROPZONE_POSITIONS_TABLE[idx];
            position_t path[1024];
            int dist;
            if(!pathfind(fromPos, tmp_pos, path, & dist)) continue;
            if( dist < min_distance){
                bestPos = tmp_pos;
                min_distance = dist;
                bestDropZone = idx;
            }
        }
    }
    if(position_equals(fromPos, bestPos)){
        return 0;
    }
    bestPos = getBestDropZonePosition(bestDropZone, true);
    return min_distance;
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



position_t toFirstStockPos(position_t targetPos){
    position_t firstPos; 
    double a_block_rad = (targetPos.a - drive.position.a) * DEG_TO_RAD;
    double offsetX = OFFSET_STOCK * (MULT_PARAM - 1) * cos(a_block_rad);
    double offsetY = OFFSET_STOCK * (MULT_PARAM - 1) * sin(a_block_rad);
    double cos_a = cos(drive.position.a * DEG_TO_RAD);
    double sin_a = sin(drive.position.a * DEG_TO_RAD);
    firstPos.a = targetPos.a;//On veut l'angle final
    firstPos.x = targetPos.x + offsetX * cos_a - offsetY * sin_a;
    firstPos.y = targetPos.y + offsetX * sin_a + offsetY * cos_a;
    return firstPos;
}