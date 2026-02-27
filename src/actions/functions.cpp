#include "actions/functions.h"
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
bool rotateBlocks(){
    static int state = 1;
    switch (state){
        case 1:
            if (resetSpinClaws() & openClaws())
                state++;
            break;
        case 2:
            if (closeClaws())
                state++;
            break;
        case 3:
            if (spinAllClaws()){
                state = 1;
                openClaws();
                return true;
            }
            break;
    }
    return false;
}

bool lowerClaws(){
    static int state = 1;
    //LOG_INFO("lowerClaws state = ", state);
    static unsigned long startTime = 0;
    switch (state){
        case 1:
            arduino.moveMotorDC(50, true);
            startTime = _millis();
            state++;
            break;
        case 2:
            if (readLimitSwitchBottom() || (_millis() >= startTime + 2000)){ // Si pinces bloquées ou après 2s
                startTime = _millis();
                state++;
            }
            break;
        case 3:
            if (_millis() >= startTime + 500){
                arduino.stopMotorDC();
                state = 1;
                return true;
            }
            break;
    }
    return false;
}

bool raiseClaws(){
    static int state = 1;
    switch (state){
        case 1:
            arduino.moveMotorDC(110, false);
            state++;
            break;
        case 2:
            if (readLimitSwitchTop()){
                arduino.stopMotorDC();
                state = 1;
                return true;
            }
            break;
    }
    return false;
}


bool rotateTwoBlocks(){
    static int state = 1;
    static int choice;
    switch (state){
        case 1 :
            if (closeClaws()){
                choice = rand() % 6;
                state++;
            }
            break;
        case 2:
            if (raiseClaws())
                state++;
            break;
        case 3:
            switch(choice){
                case 0: if (spinClaws(true,  true,  false, false)) state++; break;
                case 1: if (spinClaws(true,  false, true,  false)) state++; break;
                case 2: if (spinClaws(true,  false, false, true )) state++; break;
                case 3: if (spinClaws(false, true,  true,  false)) state++; break;
                case 4: if (spinClaws(false, true,  false, true )) state++; break;
                case 5: if (spinClaws(false, false, true,  true )) state++; break;
            }
            break;
        case 4:
            state = 1;
            return true;
            break;
    }
    return false;
}

bool dropBlock(){
    static int state = 0;
    static unsigned long startTime = 0;
    switch (state){
        case 0:
            startTime = _millis();
            state++;
            break;
        case 1:
            if (lowerClaws()) // Si pinces bloquées
                state++;
            break;
        case 2:
            if (openClaws() & resetSpinClaws() & raiseClaws()){
                state = 0;
                return true;
            }
            break;
    }
    return false;
}


// ------------------------------------------------------
//                   SERVO CONTROL
// ------------------------------------------------------


bool closeClaws(){
    return snapClaws(true);
}
bool openClaws(){
    return snapClaws(false);
}

bool snapClaws(bool closed){
    static bool prevState = !closed;
    int target = closed ? 16 : 130;
    if (prevState != closed){
        arduino.moveServoSpeed(SERVO_CLAW_CLOSE_1, target, 100);
        prevState = closed;
    }
    int current = 0;
    if (!arduino.getServo(SERVO_CLAW_CLOSE_1, current)) return false;
    return current == target;
}

bool resetSpinClaws(){
    return spinClaws(false, false, false, false);
}
bool spinAllClaws(){
    return spinClaws(true, true, true, true);
}

bool spinClaws(bool spin1, bool spin2, bool spin3, bool spin4){
    const int speed = 200;
    static bool prevSpin1 = !spin1;
    static bool prevSpin2 = !spin2;
    static bool prevSpin3 = !spin3;
    static bool prevSpin4 = !spin4;
    int target1 = spin1 ? 180 : 0;
    int target2 = spin2 ? 0 : 180;
    int target3 = spin3 ? 180 : 0;
    int target4 = spin4 ? 0 : 180;
    if (prevSpin1 != spin1){
        arduino.moveServoSpeed(SERVO_SPIN_1, target1, speed);
        prevSpin1 = spin1;
    }
    if (prevSpin2 != spin2){
        arduino.moveServoSpeed(SERVO_SPIN_2, target2, speed);
        prevSpin2 = spin2;
    }
    if (prevSpin3 != spin3){
        arduino.moveServoSpeed(SERVO_SPIN_3, target3, speed);
        prevSpin3 = spin3;
    }
    if (prevSpin4 != spin4){
        arduino.moveServoSpeed(SERVO_SPIN_4, target4, speed);
        prevSpin4 = spin4;
    }
    bool done1 = true, done2 = true, done3 = true, done4 = true;
    int current = 0;
    if (!arduino.getServo(SERVO_SPIN_1, current)) 
        return false; 
    done1 = (current == target1);
    if (!arduino.getServo(SERVO_SPIN_2, current)) 
        return false; 
    done2 = (current == target2);
    if (!arduino.getServo(SERVO_SPIN_3, current)) 
        return false; 
    done3 = (current == target3);
    if (!arduino.getServo(SERVO_SPIN_4, current)) 
        return false; 
    done4 = (current == target4);
    return (done1 && done2 && done3 && done4);
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
    return ( resetSpinClaws() & openClaws());
}
void enableActuators(){
    for (int i = 0; i < 4; i++){
        arduino.enableStepper(i);
    }
    arduino.enableServos();
    drive.enable();
}
void disableActuators(){
    stopTribuneElevator();
    for (int i = 0; i < 4; i++){
        arduino.disableStepper(i);
    }
    arduino.disableServos();
    drive.disable();
}


// ------------------------------------------------------
//                        OTHER
// ------------------------------------------------------

int GetBestDropZone(position_t fromPos){
    int bestDropZone = -1;
    double bestDist2 = 1000000;

    for (int i = 0; i < DROPZONE_COUNT; i++){
        if (tableStatus.dropzone_states[i] != TableState::DROPZONE_EMPTY)
            continue;

        position_t dropzonePos = DROPZONE_POSITIONS_TABLE[i];
        double dist2 = position_distance(fromPos, dropzonePos) + abs( tableStatus.colorTeam == BLUE ? dropzonePos.y - 1500 : dropzonePos.y + 1500); // We want to favor the dropzones on our side of the table

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

        double dist2 = position_distance(stockOff,targetPos);

        if (dist2 < bestDist2){
            bestDist2 = dist2;
            bestOff = offNum;
        }
    }

    return bestOff;
}

position_t getBestDropZonePosition(int dropzoneNum, position_t fromPos){
    position_t dropzonePos = DROPZONE_POSITIONS_TABLE[dropzoneNum];
    position_t vect = position_vector(dropzonePos, fromPos);
    position_normalize(vect);
    position_t bestPoss = position_t{dropzonePos.x + int(vect.x * OFFSET_DROPZONE), dropzonePos.y + int(vect.y * OFFSET_DROPZONE), RAD_TO_DEG * position_angle(fromPos, dropzonePos)};
    return bestPoss;
}

void setStockAsRemoved(int num){
    tableStatus.avail_stocks[num] = false;
    LOG_INFO("Removed stock ", num);
}

void setDropzoneState(int dropzoneNum, TableState::dropzone_state_t state){
    tableStatus.dropzone_states[dropzoneNum] = state;
    LOG_INFO("Set dropzone ", dropzoneNum, " state to ", state);
}

void setDropzoneAsError(int dropzoneNum){
    setDropzoneState(dropzoneNum, TableState::DROPZONE_ERROR);
}

bool returnToHome(){
    unsigned long time = _millis() - tableStatus.startTime;
    position_t homePos;
    homePos.x = (time < 98000) ? -200 : -600;
    homePos.y = (tableStatus.colorTeam == BLUE) ? 1100 : -1100;
    homePos.a = 180;
    nav_return_t res = navigationGoTo(homePos, true);
    return res == NAV_DONE && isRobotInArrivalZone(drive.position);
}

// Function to check if a point (px, py) lies inside the rectangle
bool m_isPointInsideRectangle(float px, float py, float cx, float cy, float w, float h) {
    float left = cx - w / 2, right = cx + w / 2;
    float bottom = cy - h / 2, top = cy + h / 2;
    return (px >= left && px <= right && py >= bottom && py <= top);
}

void opponentInAction(position_t position){
    for(int i = 0; i < STOCK_COUNT; i++){
        position_t stock_pos = STOCK_POSITIONS_TABLE[i];
        if (tableStatus.avail_stocks[i] && m_isPointInsideRectangle(position.x, position.y, stock_pos.x, stock_pos.y, 
            OPPONENT_ROBOT_RADIUS * 2 + (stock_pos.a == 90 ? STOCKS_LENGTH : STOCKS_WIDTH), 
            OPPONENT_ROBOT_RADIUS * 2 + (stock_pos.a == 90 ? STOCKS_WIDTH : STOCKS_LENGTH)) )// we  consider stock orientation
        {
            LOG_INFO("Opponent in action at stock ", i, " at position ", position.x, " / ", position.y);
            tableStatus.avail_stocks[i] = false;
            return;
        }
    }
    for(int i = 0; i < DROPZONE_COUNT; i++){
        position_t dropzone_pos = DROPZONE_POSITIONS_TABLE[i];
        if (tableStatus.dropzone_states[i] == TableState::DROPZONE_EMPTY && m_isPointInsideRectangle(position.x, position.y, dropzone_pos.x, dropzone_pos.y, OPPONENT_ROBOT_RADIUS * 2 + DROPZONE_WIDTH, OPPONENT_ROBOT_RADIUS * 2 + DROPZONE_LENGTH))
        {
            LOG_INFO("Opponent at dropzone ", i ," at position ", position.x, position.y);
            tableStatus.dropzone_states[i] = (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_YELLOW : TableState::DROPZONE_BLUE;
            return;
        }
    }
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