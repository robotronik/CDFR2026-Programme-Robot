#include "actions/strats.hpp"
#include "actions/functions.h"
#include "utils/logger.hpp"
#include "defs/structs.hpp"
#include "navigation/driveControl.h"
#include <math.h>
#include "main.hpp" // for tableStatus

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
        //LOG_INFO("Next stock to take: ", closest_stock);
        return closest_stock;
    }
}

bool chooseStockStrategy(int& stockNum, int& stockOffset){
    // TODO check if stock is available
    // Returns true if the robot can take a stock
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
    //LOG_WARNING("No stock available");
    return false;
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
        double dist2 = position_distance(fromPos, dropzonePos) + fabs( tableStatus.colorTeam == BLUE ? dropzonePos.y - 1500 : dropzonePos.y + 1500); // We want to favor the dropzones on our side of the table

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestDropZone = i;
        }
    }

    return bestDropZone;
}

int getBestStockPositionOff(int stockNum, position_t fromPos){
    int bestOff = -1;
    double bestDist2 = INFINITY;

    position_t stockPos = STOCK_POSITIONS_TABLE[stockNum];

    for (int i = 0; i < 2; i++){
        int offNum = STOCK_OFFSET_MAPPING[stockNum][i];
        if (offNum == -1)
            continue;

        position_t stockOff = STOCK_OFFSETS[offNum];
        position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, 0};

        double dist2 = position_distance(fromPos,targetPos);

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestOff = offNum;
        }
    }

    return bestOff;
}

position_t getBestDropZonePosition(int dropzoneNum, position_t fromPos){
    if (dropzoneNum == 7 || dropzoneNum == 4 || dropzoneNum == 2 ){// deactivated for now
        position_t bestPoss = DROPZONE_POSITIONS_TABLE[dropzoneNum];
        /*
        position_t vect = position_vector(dropzonePos, fromPos);
        position_normalize(vect);
        position_t bestPoss = position_t{dropzonePos.x + int(vect.x * OFFSET_DROPZONE), dropzonePos.y + int(vect.y * OFFSET_DROPZONE), RAD_TO_DEG * position_angle(fromPos, dropzonePos)};
        */
        if(position_distance(fromPos, position_sum(bestPoss, position_t{.x = OFFSET_DROPZONE, .y=0}))
            < position_distance(fromPos, position_sum(bestPoss, position_t{.x = - OFFSET_DROPZONE, .y=0}))){
                bestPoss = position_sum(bestPoss, position_t{.x = OFFSET_DROPZONE, .y=0});
                bestPoss.a = 180;
        }else{
            bestPoss = position_sum(bestPoss, position_t{.x = -OFFSET_DROPZONE, .y=0});
            bestPoss.a = 0;
        }
        return bestPoss;
    }else{
        position_t bestPoss = DROPZONE_POSITIONS_TABLE[dropzoneNum];
        if(MAX_WIDTH_TABLE - abs(bestPoss.x) < MAX_LENGTH_TABLE - abs(bestPoss.y)){
            bestPoss.x += (bestPoss.x > 0? -1 : 1 ) * OFFSET_DROPZONE;
            bestPoss.a = (bestPoss.x > 0? 0 : 180);
        }else{
            bestPoss.y += (bestPoss.y > 0? -1 : 1 ) * OFFSET_DROPZONE;
            bestPoss.a = (bestPoss.y > 0? 90 : -90);
        }
        return bestPoss;
    }
    
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
    double offsetX = OFFSET_STOCK * MULT_PARAM * cos(a_block_rad);
    double offsetY = OFFSET_STOCK * MULT_PARAM * sin(a_block_rad);
    double cos_a = cos(drive.position.a * DEG_TO_RAD);
    double sin_a = sin(drive.position.a * DEG_TO_RAD);
    firstPos.a = targetPos.a;//On veut l'angle final
    firstPos.x = targetPos.x + offsetX * cos_a - offsetY * sin_a;
    firstPos.y = targetPos.y + offsetX * sin_a + offsetY * cos_a;
    return firstPos;
}