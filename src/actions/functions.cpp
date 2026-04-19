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

bool lowerClaws(){
    static int state = 1;
    static unsigned long startTime = 0;
    static unsigned long startTime2 = 0;

    switch (state){
        case 1:
            LOG_DEBUG("Lower Claws");
            if (readLimitSwitchBottom()){
                state = 1;
                return true;
            }
            arduino.moveMotorDC(100, true);
            startTime = _millis();
            startTime2 = 0;
            state = 2;
            break;
        case 2:
            if (readLimitSwitchBottom() && startTime2 == 0) 
                startTime2 = _millis();  // démarre une seule fois
    
            if (_millis() - startTime >= 1000 ||
                (startTime2 != 0 && _millis() - startTime2 >= 150)) {
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
    static unsigned long startTime = 0;

    switch(state){
        case 1: // montée rapide
            LOG_DEBUG("Raise Claws");
            if (readLimitSwitchTop()){
                arduino.keepMotorDCup();
                state = 1;
                return true;
            }
            arduino.moveMotorDC(120, false);
            startTime = _millis();
            state = 2;
            break;
        case 2: // approche lente jusqu'au switch
            if (readLimitSwitchTop() || (_millis() - startTime >= 2000)){ // timeout de sécurité
                arduino.keepMotorDCup();        // maintenir la pince levée
                state = 1;
                return true;
            }
            break;
    }

    return false;
}

bool rotateTwoBlocksDefault(){
    bool order[4]{false,false,false,false};
    return rotateTwoBlocks(order);
}

bool rotateTwoBlocks(bool *order){  
    static int state = 1;
    static unsigned long startTime = 0;
    switch (state){
        case 1:
            if (closeClaws() && raiseClaws()){
                state = 2;
                startTime = _millis();
            }
            break;
        case 2:

            bool any;
            if (tableStatus.colorTeam == colorTeam_t::YELLOW){
                any = order[0] || order[1] || order[2] || order[3] ;//if jaune
            }else{
                any = !(order[0] && order[1] && order[2] && order[3]); // if bleu
            }
            bool a=false,b=false,c=false,d=false;

            if (any){
                if (tableStatus.colorTeam == colorTeam_t::BLUE){
                    a = !order[0]; b = !order[1]; c = !order[2]; d = !order[3];
                }else{
                    a = order[0]; b = order[1]; c = order[2]; d = order[3];
                }
                
            }
            if (!any || spinClaws(a,b,c,d) || _millis() - startTime >= 150){ // Attendre bloc tourne un peu
                state = 1;
                return true;
            }
            break;
    }
    return false;
}

bool dropBlock(){
    static int state = 0;
    switch (state){
        case 0:
            LOG_DEBUG("Drop Block");
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

bool enableCursor(bool enable){
    int target = enable ? 150 : 90;

    arduino.moveServo(SERVO_NUM_7, 90);
    arduino.moveServo(SERVO_NUM_6, target);

    int current = 0;
    if (!arduino.getServo(SERVO_NUM_6, current)) 
        return false;

    return current == target;
}

bool flipOneBlock(){
    static int state = 0;
    static unsigned long startTime = 0;
    //servo 7 : 90 = bras haut / 150 = intermédiaire / 180 = bras bas
    //servo 8 : 170 = ventouse bas,  0 = ventouse haut 
    switch(state){
        case 0: //take block
            if ( arduino.writeSensor(1, true) && moveServoAndWait(SERVO_NUM_7,180,200) && moveServoAndWait(SERVO_NUM_6,175,200)){
                startTime = _millis();
                state = 1;
            }
            break;

        case 1: //monte bras
            if (_millis() > startTime + 500){
                if (moveServoAndWait(SERVO_NUM_6,150,200))
                    state = 2;
            }
            break;

        case 2: // tourne ventouse
            if (moveServoAndWait(SERVO_NUM_7,0,200))
                state = 3;
            break;

        case 3: //baisse bras
            if (moveServoAndWait(SERVO_NUM_6,180,200)){
                startTime = _millis();
                state = 4;
            }
            break;
        case 4: //lacher ventouse
            if (arduino.writeSensor(1, false) &&  _millis() > startTime + 1000){
                if (moveServoAndWait(SERVO_NUM_6,80,200) & moveServoAndWait(SERVO_NUM_7,180,200)){
                    state = 0; 
                    return true;
                }
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
    return snapClaws(closed, true);
}

bool snapClaws(bool closed, bool small){
    static int prevTarget = -1;
    int target = closed ? 10 : (small ? 45 : 130);
    if (prevTarget != target){
        arduino.moveServoSpeed(SERVO_CLAW_CLOSE_1, target, 150);
        prevTarget = target;
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
    arduino.moveServo(SERVO_NUM_6, 90);
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
    unsigned long time = _millis() - tableStatus.startTime;
    position_t homePos;
    homePos.x = (time < 98000) ? -200 : -600;
    homePos.y = (tableStatus.colorTeam == BLUE) ? 1200 : -1200;
    homePos.a = 180;
    raiseClaws();
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
        if (tableStatus.avail_stocks[i] && m_isPointInsideRectangle(position.x, position.y, stock_pos.x, stock_pos.y, OPPONENT_ROBOT_RADIUS * 2, OPPONENT_ROBOT_RADIUS * 2)){
            LOG_INFO("Opponent in action at stock ", i, " at position ", position.x, " / ", position.y);
            tableStatus.avail_stocks[i] = false;
            return;
        }
    }
    for(int i = 0; i < DROPZONE_COUNT; i++){
        position_t dropzone_pos = DROPZONE_POSITIONS_TABLE[i];
        if (m_isPointInsideRectangle(position.x, position.y, dropzone_pos.x, dropzone_pos.y, OPPONENT_ROBOT_RADIUS + DROPZONE_WIDTH, OPPONENT_ROBOT_RADIUS + DROPZONE_LENGTH))
        {
            LOG_INFO("Opponent at dropzone ", i ," at position ", position.x, position.y);
            tableStatus.dropzone_states[i] = (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_YELLOW : TableState::DROPZONE_BLUE;
            //TODO to remove, search dropzone for steal with other way
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