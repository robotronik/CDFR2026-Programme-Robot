#pragma once
#include "main.hpp"
#include "utils/logger.hpp"

inline void check(colorTeam_t color, int strategy){
    // Check if the color and strategy are valid
    if (color == NONE || strategy < 1 || strategy > 4)
        LOG_ERROR("Invalid color (", color, ") or strategy (", strategy, ")");
}

// Function to handle the strategy
inline position_t StratStartingPos(){
    // Returns the starting position of the robot
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);
    position_t pos = {-775, 1125, 0};

    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}

inline bool chooseStockStrategy(int& stockNum, int& stockOffset){
    // TODO check if stock is available
    // Returns true if the robot can take a stock
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);

    int todo_stocks[9];
    int num;
    
    switch (strategy)
    {
    case 1:
        todo_stocks[0] = 0;
        todo_stocks[1] = 1;
        num = 2;
    break;
    
    }
    if (color == YELLOW){
        for (int i = 0; i < num; i++){
            if (i != 4 && i != 8) // inverse pas les stocks 4 et 8 qui sont au milieu
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
    LOG_GREEN_INFO("No stock available");
    return false;
}

inline int chooseNextStock(){
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