#include "actions/strats.hpp"
#include "actions/functions.h"

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
    position_t pos = {-775, 1125, 0};

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

position_t calculateClosestArucoPosition(position_t currentPos, position_t& outPos){
    outPos = currentPos;
    position_t closestPos = ARUCO_POSITIONS_TABLE[0];
    double dist20 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[0]);
    double dist21 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[1]);
    double dist22 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[2]);
    double dist23 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[3]);
    double minDistance = dist20;
    if (dist21 < minDistance){
        minDistance = dist21;
        closestPos = ARUCO_POSITIONS_TABLE[1];
    }
    if (dist22 < minDistance){
        minDistance = dist22;
        closestPos = ARUCO_POSITIONS_TABLE[2];
    }
    if (dist23 < minDistance){
        minDistance = dist23;
        closestPos = ARUCO_POSITIONS_TABLE[3];
    }
    // Check if we are above the aruco marker
    const double minimal_distance = 200.0; // 20cm
    if (minDistance < minimal_distance){
        LOG_WARNING("Above aruco marker, need to move away first");
        double displacement = minimal_distance - minDistance + 1; //+margin
        position_t tmp = position_vector(closestPos, currentPos);
        position_normalize(tmp);
        tmp.x *= displacement;
        tmp.y *= displacement;
        outPos.x += tmp.x;
        outPos.y += tmp.y;
    }
    outPos.a = RAD_TO_DEG * position_angle(drive.position, closestPos) + OFFSET_CAM_A;

    return closestPos;
}

int GetBestDropZone(position_t fromPos){
    int bestDropZone = -1;
    double bestDist2 = 1000000;

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
    if (dropzoneNum == 7 || dropzoneNum == 4 || dropzoneNum == 2 ){
        position_t dropzonePos = DROPZONE_POSITIONS_TABLE[dropzoneNum];
        position_t vect = position_vector(dropzonePos, fromPos);
        position_normalize(vect);
        position_t bestPoss = position_t{dropzonePos.x + int(vect.x * OFFSET_DROPZONE), dropzonePos.y + int(vect.y * OFFSET_DROPZONE), RAD_TO_DEG * position_angle(fromPos, dropzonePos)};
        return bestPoss;
    }else{
        position_t bestPoss = DROPZONE_POSITIONS_TABLE[dropzoneNum];
        if(bestPoss.x - (bestPoss.x > 0 ? MAX_WIDTH_TABLE : -MAX_WIDTH_TABLE) < bestPoss.y - (bestPoss.y > 0 ? MAX_LENGTH_TABLE : -MAX_LENGTH_TABLE)){
            bestPoss.x += (bestPoss.x > 0? -1 : 1 ) * OFFSET_DROPZONE;
            bestPoss.a = (bestPoss.x > 0? 0 : 180);
        }else{
            bestPoss.y += (bestPoss.y > 0? -1 : 1 ) * OFFSET_DROPZONE;
            bestPoss.a = (bestPoss.y > 0? 90 : -90);
        }
        return bestPoss;
    }
    
}