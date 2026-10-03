#include "defs/structs.hpp"
#include "drive_interface.h"
#include "navigation/navigation.h"
#include "i2c/Arduino.hpp"
#include "actions/strats.hpp"
#include "main.hpp"
#include "utils/logger.hpp"
#include <math.h>



// ------------------------------------------------------
//                        OTHER
// ------------------------------------------------------

bool returnToHome(){

    static position_t homePos;
    nav_return_t res = navigation.goTo(homePos, true);
    if (res == NAV_ERROR){
        LOG_ERROR("RETURN_TO_HOME: Navigation error");
        homePos.y += (tableStatus.colorTeam == BLUE) ? 50 : -50; // recule un peu et retente
    }
    if (res == NAV_DONE){
        LOG_GREEN_INFO("RETURN_TO_HOME: Done");
        return true;
    }
    return false;
}

// Function to check if a point (px, py) lies inside the rectangle
bool m_isPointInsideRectangle(float px, float py, float cx, float cy, float w, float h) {
    float left = cx - w / 2, right = cx + w / 2;
    float bottom = cy - h / 2, top = cy + h / 2;
    return (px >= left && px <= right && py >= bottom && py <= top);
}

void opponentInAction(position_t position){
    if (position_equals(position, position_t{.x=0, .y=0, .a=0})){
        return; // useless code to get rid of warning 
    }
    // TODO implement this function
    /* Detect from position of adversary the action of the adversary */
}

void switchTeamSide(colorTeam_t color){ // TODO moove to tableState
    if (color == NONE) return;
    if (currentState == RUN) return;
    if (color != tableStatus.colorTeam){
        LOG_INFO("Color switch detected");
        tableStatus.colorTeam = color;

        switch (color)
        {
        case BLUE:
            LOG_INFO("Switching to BLUE");
            arduino.RGB_Blinking(0, 0, 255);
            break;
        case YELLOW:
            LOG_INFO("Switching to YELLOW");
            arduino.RGB_Blinking(255, 56, 0);
            break;
        default:
            break;
        }

        position_t pos = StratStartingPos(&tableStatus);
        drive.setCoordinates(pos);
        navigation.goTo(pos, true, true); // Go to starting pos with A* and slow mode to avoid collisions during the switch
    }
}

void switchStrategy(int strategy){ // TODO moove to tableState
    if (currentState == RUN) return;
    if (strategy < 1 || strategy > 4){
        LOG_ERROR("Invalid strategy");
        return;
    }
    if (strategy != tableStatus.strategy){
        LOG_INFO("Strategy switch detected");
        tableStatus.strategy = strategy;
        position_t pos = StratStartingPos(&tableStatus);
        drive.setCoordinates(pos);
    }
}

bool isRobotInArrivalZone(position_t position){
    // Returns true if the robot is in the arrival zone
    int robotSmallRadius = 100;
    int w = 450;
    int h = 600;
    int c_x = -550 - w/2;
    int c_y = tableStatus.colorTeam == BLUE ? (900 + h/2) : (-900 - h/2);
    return m_isPointInsideRectangle(position.x, position.y, c_x, c_y, w + 2*robotSmallRadius, h + 2*robotSmallRadius);
}

