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
    position_t pos = {-675, 1125, 120};

    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}

double chooseNextStock(int& closest_stock, int& closest_offset){
    // Returns the number of the closest available stock to be taken
    double min = INFINITY;
    closest_stock = -1;
    closest_offset = -1;
    for (int i = 0; i < STOCK_COUNT; i++){
        if (tableStatus.avail_stocks[i]){            
            for (int j = 0; j < 2; j++){
                int offNum = STOCK_OFFSET_MAPPING[i][j];
                if (offNum == -1)
                    continue;
                /*
                * la distance vers le stock est multipliée par un facteur variant entre 1 et TIME_TO_TAKE
                * le facteur TIME_TO_TAKE étant le temps de prise de l'adversaire x10 est peut-être un peu grand 
                * Réduire avec un facteur 2 pour limiter à 15 le facteur?
                */
                double dist2 =  toAStarDistStock(i, offNum);
                if(dist2 != INFINITY){
                    dist2 *= TIME_TO_TAKE / (double)tableStatus.avail_stocks[i];
                }
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
        return INFINITY;
    }else{
        return min;
    }
}

double toAStarDistStock(int stockNum, int stockOffset){
    position_t stockPos = STOCK_POSITIONS_TABLE[stockNum];
    position_t stockOff = STOCK_OFFSETS[stockOffset];
    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);
    position_t target = position_sum(stockPos, stockOff);
    LOG_EXTENDED_DEBUG("Calculating dist A* to stock: ", stockNum, " at pos { ",target.x, ", ", target.y,", ", target.a,"}");
    target.a = angle;
    return toAStarDist(target);
}

double toAStarDist(position_t a){
    double length;
    position_t path[1024];

    if (pathfind(drive.position, a, path, length) <= 0)
        return INFINITY;

    double delta = a.a - drive.position.a;

    while (delta > 180) delta -= 360;
    while (delta < -180) delta += 360;

    double cost = length + 100.0 * fabs(DEG_TO_RAD * delta);

    // pénalité si on va côté adverse
    if ((tableStatus.colorTeam == BLUE && a.y < drive.position.y) ||
        (tableStatus.colorTeam != BLUE && a.y > drive.position.y)) {
        cost += 20.0;
    }

    return cost;
}

double chooseStockStrategy(int& stockNum, int& stockOffset){
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    static int i;
    check(color, strategy);

    int todo_stocks[9];
    int num = 0;
    LOG_EXTENDED_DEBUG("Strategy", strategy);
    switch (strategy)
    {   
        case 1:
            break;
        case 2:
            todo_stocks[0] = 5;
            todo_stocks[1] = 7;
            num = 2;
            break;
        case 3:
            todo_stocks[0] = 3;
            num = 1;
            break;
        case 4:
            break;
    }

    if (color == YELLOW){
        for (int i = 0; i < num; i++){
            todo_stocks[i] = (todo_stocks[i] + STOCK_COUNT/2) % STOCK_COUNT;
        }
    }
    if (_millis() < tableStatus.startTime + 500)
        i = 0;
    while (i < num){
        if (tableStatus.avail_stocks[todo_stocks[i]]){
            stockNum = todo_stocks[i];
            /*
            * la distance vers le stock est multipliée par un facteur variant entre 1 et TIME_TO_TAKE
            * le facteur TIME_TO_TAKE étant le temps de prise de l'adversaire x10 est peut-être un peu grand 
            * Réduire avec un facteur 2 pour limiter à 15 le facteur?
            */
            double dist =  getBestStockPositionOff(stockNum, stockOffset);
            if(dist != INFINITY){
                dist *= TIME_TO_TAKE / (double)tableStatus.avail_stocks[todo_stocks[i]];
            }else{
                i++;
                continue; // pas break sinon tu casses toute la stratégie
            }
            LOG_INFO("Best stock to take: ", stockNum);
            return dist;
        }
        i++;
    }

    double dist = chooseNextStock(stockNum, stockOffset); // Choose the closest stock if the strategy stocks are not available
    if (stockNum != -1){
        LOG_INFO("Best stock to take: ", stockNum, " at dist: ", dist);
        return dist;
    }
    //LOG_WARNING("No stock available");
    return INFINITY;
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
    const double target_distance_min = 350.0; // mm
    const double target_distance_max = 550.0; // mm

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

double getBestStockPositionOff(int stockNum, int& bestOff){
    bestOff = -1;
    double bestDist = INFINITY;

    for (int i = 0; i < 2; i++){
        int offNum = STOCK_OFFSET_MAPPING[stockNum][i];
        if (offNum == -1){
            continue;
        }

        double dist2 = toAStarDistStock(stockNum, offNum);

        if (dist2 < bestDist){
            bestDist = dist2;
            bestOff = offNum;
        }
    }

    return bestDist; // INFINITY si échec
}

double getBestDropZonePosition(int& dropzoneNum, position_t& bestPoss, bool steal){
    //Changer le 0 pour décaler le robot sur les zones de dépose
    const int DROPZONE_OFFSET = 0;

    double dropZoneOffset = OFFSET_DROPZONE;
    TableState::dropzone_state_t zone_of_interest = TableState::DROPZONE_EMPTY;
    if(steal){
        dropZoneOffset = OFFSET_STOCK*1.18;
        zone_of_interest = (tableStatus.colorTeam == YELLOW ? TableState::DROPZONE_BLUE : TableState::DROPZONE_YELLOW);
    }

    double min = INFINITY;
    dropzoneNum = -1;

    double d1;
    position_t temp_pos;

    for(int k = 0; k< DROPZONE_COUNT; k++){
        if(tableStatus.dropzone_states[k] != zone_of_interest || ( !steal && (k == 3 || k == 8 || k == 10 || k == 11))){ // Don't drop on zone == 3 or 8 or 10 or 11
            continue;
        }else{
            if (k == 7 || k == 4 || k == 2 ){// DropZone centrales
                temp_pos = DROPZONE_POSITIONS_TABLE[k];

                d1 = toAStarDist(position_sum(temp_pos, position_t{.x = dropZoneOffset, .y= - DROPZONE_OFFSET})); 
                double d2 = toAStarDist(position_sum(temp_pos, position_t{.x = -1 * dropZoneOffset, .y= DROPZONE_OFFSET}));

                if(d1 < d2 ){
                    temp_pos = position_sum(temp_pos, position_t{.x = dropZoneOffset, .y= - DROPZONE_OFFSET});
                    temp_pos.a = 180;
                }else{
                    temp_pos = position_sum(temp_pos, position_t{.x =  -1 * dropZoneOffset, .y= DROPZONE_OFFSET});
                    temp_pos.a = 0;
                    d1 = d2;
                }

            }else{// DropZone latérale
                temp_pos = DROPZONE_POSITIONS_TABLE[k];
                if(MAX_WIDTH_TABLE - abs(temp_pos.x) < MAX_LENGTH_TABLE - abs(temp_pos.y)){
                    double signeX = (temp_pos.x > 0? 1 : -1 );
                    temp_pos.x += -1 * signeX * dropZoneOffset;
                    temp_pos.y += signeX * DROPZONE_OFFSET;
                    temp_pos.a = (signeX > 0? 0 : 180);
                }else{
                    double signeY = (temp_pos.y > 0? -1 : 1 );
                    temp_pos.y += signeY * dropZoneOffset;
                    temp_pos.x += signeY * DROPZONE_OFFSET;
                    temp_pos.a = (signeY < 0? 90 : -90);
                }
                d1 = toAStarDist(temp_pos);
            }

            /*
            * la distance vers le stock est multipliée par un facteur variant entre 1 et TIME_TO_DROP
            * le facteur TIME_TO_DROP étant le temps de prise de l'adversaire x10 est peut-être un peu grand 
            * Réduire avec un facteur 2 pour limiter à 15 le facteur?
            */
            if(steal && d1 != INFINITY){
                d1 *= (double)TIME_TO_DROP /  (double)tableStatus.dropzone_proba[k];
            }
            if(min > d1){
                bestPoss = temp_pos;
                dropzoneNum = k;
                min = d1;
            }
        }
    }
    return (dropzoneNum != -1 ? min : INFINITY);
}

void respawnGranaryZone(int zone, int flagIndex, TableState::dropzone_state_t etat){
    if(!tableStatus.granaryAlreadyTaken[flagIndex]){
        tableStatus.dropzone_states[zone] = etat;
        tableStatus.dropzone_proba[zone] = TIME_TO_DROP;
    }
}

void updateGranaryStock(){
    static int wave = 0;
    double time = _millis() - tableStatus.startTime;

    if ((wave == 0 && time > 50000) || (wave == 1 && time > 70000) || (wave == 2 && time > 90000)){
        TableState::dropzone_state_t etat = (tableStatus.colorTeam == YELLOW) ? tableStatus.DROPZONE_BLUE : tableStatus.DROPZONE_YELLOW;
        respawnGranaryZone(3 ,0, etat);
        respawnGranaryZone(8 ,1, etat);
        respawnGranaryZone(10,2, etat);
        respawnGranaryZone(11,3, etat);
        wave++;
    }
}
/*
    Determine the best drop zone from wich to steal
    For now very simple 
    TODO: developped with adversary position
*/
double getBestStealZonePosition(int& bestDropZone, position_t& bestPos){
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


bool NearestValidZone(position_t* pos){
    const float MARGIN = 400.0;
    const float X_MIN = -550.0 + MARGIN, X_MAX = 1000.0 - MARGIN;
    const float Y_MIN = -1500.0 + MARGIN, Y_MAX = 1500.0 - MARGIN;
    bool modified = false;

    bool hitLeft = false, hitRight = false, hitBottom = false, hitTop = false;

    // Clamp X
    if (pos->x < X_MIN){
        pos->x = X_MIN;
        hitBottom = true;
        modified = true;
        LOG_ERROR("Hit bottom wall");
    }
    else if (pos->x > X_MAX){
        pos->x = X_MAX;
        hitTop = true;
        modified = true;
        LOG_ERROR("Hit top wall");
    }

    // Clamp Y
    if (pos->y < Y_MIN){
        pos->y = Y_MIN;
        hitRight = true;
        modified = true;
        LOG_ERROR("Hit right wall");
    }
    else if (pos->y > Y_MAX){
        pos->y = Y_MAX;
        hitLeft = true;
        modified = true;
        LOG_ERROR("Hit left wall");
    }

    if (modified){
        //Corriger l'angle pour regarder le mur
        if (hitLeft)        pos->a = 90.0;     // regarde vers +Y
        else if (hitRight)  pos->a = -90.0;   // regarde vers -Y
        else if (hitBottom) pos->a = 180.0;    // regarde vers +X
        else if (hitTop)    pos->a = 0.0;   // regarde vers -X
        LOG_WARNING("Position clamped + angle corrected: (", pos->x, ", ", pos->y, ", ", pos->a, ")");
    }

    return modified;
}