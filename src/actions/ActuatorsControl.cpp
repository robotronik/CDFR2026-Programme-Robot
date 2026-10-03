#include "defs/constante.h"
#include "actions/ActuatorsControl.hpp"

ActuatorsControl::ActuatorsControl(Arduino* arduino, DriveControl* drive) : arduino(arduino), drive(drive) {
    // Constructor implementation
}
// ------------------------------------------------------
//                   BASIC FSM CONTROL
// ------------------------------------------------------

/* Code for basic FSM (elemental actions)*/

// ------------------------------------------------------
//                   SERVO CONTROL
// ------------------------------------------------------

// Exemple of servo control
bool ActuatorsControl::moveServoAndWait(int servo, int target, int speed){
    static int prevServo = -1;
    static int prevTarget = -1;

    if (servo != prevServo || target != prevTarget){
        arduino->moveServoSpeed(servo, target, speed);
        prevServo = servo;
        prevTarget = target;
    }

    int s;
    if (!arduino->getServo(servo, s)) return false;

    return s == target;
}

// ------------------------------------------------------
//                   STEPPER CONTROL
// ------------------------------------------------------

// Exemple of stepper control
// Moves the platforms elevator to a predefined level
// 0:startpos, 1:lowest, 2:Banner, 3:highest
bool ActuatorsControl::moveColumnsElevator(int level){
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
        arduino->moveStepper(target, STEPPER_NUM_2);
    }
    int32_t currentValue;
    if (!arduino->getStepper(currentValue, STEPPER_NUM_2)) return false; // TODO Might need to change this (throw error)
    return (currentValue == target);
}


// ------------------------------------------------------
//                GLOBAL SET/RES CONTROL
// ------------------------------------------------------


// Returns true if actuators are home
bool ActuatorsControl::homeActuators(){
    return true; // TODO
}

void ActuatorsControl::enableActuators(){
    for (int i = 0; i < 4; i++){
        arduino->enableStepper(i);
    }
    arduino->enableServos();
    drive->enable();
}

void ActuatorsControl::disableActuators(){
    arduino->stopMotorDC();
    for (int i = 0; i < 4; i++){
        arduino->disableStepper(i);
    }
    arduino->disableServos();
    drive->disable();
}