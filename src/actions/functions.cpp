#include "actions/functions.h"
#include "main.hpp"
#include "navigation/navigation.h"
#include "lidar/lidarAnalize.h"
#include "lidar/Lidar.hpp"
#include "defs/constante.h"
#include "i2c/Arduino.hpp"
#include "actions/strats.hpp"
#include <math.h>

// ------------------------------------------------------
//                   BASIC FSM CONTROL
// ------------------------------------------------------

// Function to deploy the banner (example)
bool deployBanner(){
    static int state = 1;
    switch (state){
        case 1:
            if (moveBannerDeploy(2))
                state++;
            break;
        case 2:
            if (moveBannerDeploy(0))
                state++;
            break;
        case 3:
            if (moveBannerDeploy(2)){
                state = 1;
                moveBannerDeploy(0);
                return true;
            }
            break;
    }
    return false;
}

// ------------------------------------------------------
//                   SERVO CONTROL
// ------------------------------------------------------

// 0 : Fully inside
// 1 : Locked in
// 2 : Fully outside
bool moveBannerDeploy(int position){
    static int prevPos = !position;
    int target;
    switch (position)
    {
        case 0:
            target = 0; break;
        case 1:
            target = 25; break;
        case 2:
            target = 110; break;
    }
    if (prevPos != position){
        arduino.moveServoSpeed(SERVO_NUM_1, target, 200);
    }
    int current = 0;
    if (!arduino.getServo(SERVO_NUM_1, current)) return false;
    return current == target;
}

// ------------------------------------------------------
//                   STEPPER CONTROL
// ------------------------------------------------------

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
//                   DC MOTOR CONTROL
// ------------------------------------------------------

// Moves the tribune elevator to a predefined level
bool moveTribuneElevator(){
    arduino.moveMotorDC(80, 30);
    return true;
}

void stopTribuneElevator(){
    arduino.stopMotorDC();
}

// ------------------------------------------------------
//                GLOBAL SET/RES CONTROL
// ------------------------------------------------------


// Returns true if actuators are home
bool homeActuators(){
    return ( moveBannerDeploy(0) );
}
void enableActuators(){
    for (int i = 0; i < 4; i++){
        arduino.enableStepper(i);
    }
    arduino.enableServos();
    asserv.set_motor_state(true);
    asserv.set_brake_state(false); 
}
void disableActuators(){
    stopTribuneElevator();
    for (int i = 0; i < 4; i++){
        arduino.disableStepper(i);
    }
    arduino.disableServos();
    asserv.stop();
    asserv.set_motor_state(false);
    asserv.set_brake_state(false);    
}


// ------------------------------------------------------
//                        OTHER
// ------------------------------------------------------

bool returnToHome(){
    unsigned long time = _millis() - tableStatus.startTime;
    position_t homePos;
    homePos.x = (time < 98000) ? -200 : -600;
    homePos.y = (tableStatus.robot.colorTeam == BLUE) ? 1100 : -1100;
    homePos.a = 180;
    nav_return_t res = navigationGoTo(homePos, Direction::SHORTEST, Rotation::SHORTEST, Rotation::SHORTEST);
    return res == NAV_DONE && isRobotInArrivalZone(tableStatus.robot.pos);
}

// Function to check if a point (px, py) lies inside the rectangle
bool m_isPointInsideRectangle(float px, float py, float cx, float cy, float w, float h) {
    float left = cx - w / 2, right = cx + w / 2;
    float bottom = cy - h / 2, top = cy + h / 2;
    return (px >= left && px <= right && py >= bottom && py <= top);
}

void opponentInAction(position_t position){
    // TODO
    // m_isPointInsideRectangle(position.x, position.y, stock_pos.x, stock_pos.y, OPPONENT_ROBOT_RADIUS * 2 + w, OPPONENT_ROBOT_RADIUS * 2 + h)
}
void switchTeamSide(colorTeam_t color){
    if (color == NONE) return;
    if (currentState == RUN) return;
    if (color != tableStatus.robot.colorTeam){
        LOG_INFO("Color switch detected");
        tableStatus.robot.colorTeam = color;

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
        asserv.set_coordinates(pos.x, pos.y, pos.a);
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
        asserv.set_coordinates(pos.x, pos.y, pos.a);
    }
}

bool isRobotInArrivalZone(position_t position){
    // Returns true if the robot is in the arrival zone
    int robotSmallRadius = 100;
    int w = 450;
    int h = 600;
    int c_x = -550 - w/2;
    int c_y = tableStatus.robot.colorTeam == BLUE ? (900 + h/2) : (-900 - h/2);
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