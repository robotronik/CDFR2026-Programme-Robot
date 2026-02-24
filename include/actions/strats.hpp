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
    position_t pos = {0, 0, 0};

    // First set the position as blue
    switch (strategy)
    {
    case 1:
        pos = {-775, -1175, 0}; break;
    case 2:
        pos = {-775, -1175, 0}; break;
    case 3:
        pos = {-775, -1175, 0}; break;
    case 4:
        pos = {-775, -1175, 0}; break;
    }
    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}

inline bool StratRun(int& stockNum, int& stockOffset){
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
        //TODO Invert the stocks, attention à 8 et 9
        for (int i = 0; i < num; i++)
            todo_stocks[i] = (todo_stocks[i] + STOCK_COUNT/2) % STOCK_COUNT;
    }
    int i = 0;
    while (i < num){
        if (tableStatus.avail_stocks[todo_stocks[i]]){
            stockNum = todo_stocks[i];
            stockOffset = 0; // TODO getBestStockPositionOff(stockNum, drive.position);
            return true;
        }
        i++;
    }
    LOG_GREEN_INFO("No stock available");
    return false;
}