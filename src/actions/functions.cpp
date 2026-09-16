#include "actions/functions.h"
#include "navigation/navigation.h"
#include "lidar/lidarAnalize.h"
#include "lidar/Lidar.hpp"
#include "defs/constante.h"
#include "i2c/Arduino.hpp"
#include "actions/strats.hpp"
#include "main.hpp"
#include "utils/logger.hpp"
#include <math.h>
// ------------------------------------------------------
//                   BASIC FSM CONTROL
// ------------------------------------------------------

/* Code for basic FSM (elemental actions)*/

// ------------------------------------------------------
//                   SERVO CONTROL
// ------------------------------------------------------

// Exemple of servo control
bool moveServoAndWait(int servo, int target, int speed){
    static int prevServo = -1;
    static int prevTarget = -1;

    if (servo != prevServo || target != prevTarget){
        arduino.moveServoSpeed(servo, target, speed);
        prevServo = servo;
        prevTarget = target;
    }

    int s;
    if (!arduino.getServo(servo, s)) return false;

    return s == target;
}

// ------------------------------------------------------
//                   STEPPER CONTROL
// ------------------------------------------------------

// Exemple of stepper control
// Moves the platforms elevator to a predefined level
// 0:startpos, 1:lowest, 2:Banner, 3:highest
bool moveColumnsElevator(int level){
    static int previousLevel = -1;

    int target = 0;
    switch (level)
    {
    case 0:
        target = 0; break;
    case 1:
        target = 6000; break;
    case 2:
        target = 8000; break;
    case 3:
        target = 20000; break;
    }
    if (previousLevel != level){
        previousLevel = level;
        arduino.moveStepper(target, STEPPER_NUM_2);
    }
    int32_t currentValue;
    if (!arduino.getStepper(currentValue, STEPPER_NUM_2)) return false; // TODO Might need to change this (throw error)
    return (currentValue == target);
}


// ------------------------------------------------------
//                GLOBAL SET/RES CONTROL
// ------------------------------------------------------


// Returns true if actuators are home
bool homeActuators(){
    return true; // TODO
}
void enableActuators(){
    for (int i = 0; i < 4; i++){
        arduino.enableStepper(i);
    }
    arduino.enableServos();
    drive.enable();
}
void disableActuators(){
    arduino.stopMotorDC();
    for (int i = 0; i < 4; i++){
        arduino.disableStepper(i);
    }
    arduino.disableServos();
    drive.disable();
}


// ------------------------------------------------------
//                        OTHER
// ------------------------------------------------------

bool returnToHome(){

    static position_t homePos;
    nav_return_t res = navigationGoTo(homePos, true);
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
    // TODO implement this function
    /* Detect from position of adversary the action of the adversary */
}

void switchTeamSide(colorTeam_t color){
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

        position_t pos = StratStartingPos();
        drive.setCoordinates(pos);
        navigationGoTo(pos, true, true); // Go to starting pos with A* and slow mode to avoid collisions during the switch
    }
}
void switchStrategy(int strategy){
    if (currentState == RUN) return;
    if (strategy < 1 || strategy > 4){
        LOG_ERROR("Invalid strategy");
        return;
    }
    if (strategy != tableStatus.strategy){
        LOG_INFO("Strategy switch detected");
        tableStatus.strategy = strategy;
        position_t pos = StratStartingPos();
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

// ------------------------------------------------------
//                    INPUT SENSOR
// ------------------------------------------------------

// Returns true if button sensor was high for the last N calls
bool readButtonSensor(){
    static int count = 0;
    bool state;
    if (!arduino.readSensor(BUTTON_SENSOR_NUM, state)) return false;
    if (state)
        count++;
    else
        count = 0;
    return (count >= 5);
}

// Returns true if the latch sensor is disconnected
bool readLatchSensor(){
    static int count = 0;
    static bool prev_state = false;
    bool state;
    if (!arduino.readSensor(LATCH_SENSOR_NUM, state)) return prev_state;
    if (!state)
        count++;
    else
        count = 0;    
    prev_state = state;
    return (count >= 5);
}

bool readLimitSwitchBottom(){
    bool state;
    if (!arduino.readSensor(LS_BOTTOM_NUM, state)) return false;
    return state;
}

bool readLimitSwitchTop(){
    bool state;
    if (!arduino.readSensor(LS_TOP_NUM, state)) return false;
    return state;
}