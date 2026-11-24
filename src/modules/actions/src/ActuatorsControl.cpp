#include "defs/constante.h"
#include "actions/ActuatorsControl.hpp"
#include "i2c/Arduino.hpp"
#include "navigation/driveControl.h"
#include "utils/logger.hpp"

ActuatorsControl::ActuatorsControl(Arduino* arduino, DriveControl* drive)
    : arduino(arduino), drive(drive) {
    // Constructor implementation
}
// ------------------------------------------------------
//                   BASIC FSM CONTROL
// ------------------------------------------------------
// TODD USELESS ?
// TODD USELESS ?
/* Code for basic FSM (elemental actions)*/

// ------------------------------------------------------
//                   SERVO CONTROL
// ------------------------------------------------------

// Exemple of servo control
bool ActuatorsControl::moveServoAndWait(int servo, int target, int speed) {
    static int prevServo = -1;
    static int prevTarget = -1;

    if (servo != prevServo || target != prevTarget) {
        arduino->moveServoSpeed(servo, target, speed);
        prevServo = servo;
        prevTarget = target;
    }

    int s;
    if (!arduino->getServo(servo, s)) return false;

    return s == target;
}

// ------------------------------------------------------
//                  PUMP CONTROL
// ------------------------------------------------------

bool ActuatorsControl::setPumpState(bool state, int pumpIdx) {
    return arduino->writeSensor(PUMP_1_NUM, state); //TODO : Change to pumpIdx when multiple pumps are used
}

bool ActuatorsControl::setElectroState(bool state, int electroIdx) {
    return arduino->writeSensor(electroIdx, state);
}
// ------------------------------------------------------
//                   STEPPER CONTROL
// ------------------------------------------------------

// Exemple of stepper control
// Moves the platforms elevator to a predefined level
// 0:startpos, 1:lowest, 2:Banner, 3:highest
bool ActuatorsControl::moveColumnsElevator(int level) {
    static int previousLevel = -1;

    int target = 0;
    switch (level) {
    case 0:
        target = 0;
        break;
    case 1:
        target = 6000;
        break;
    case 2:
        target = 8000;
        break;
    case 3:
        target = 20000;
        break;
    }
    if (previousLevel != level) {
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
bool ActuatorsControl::homeActuators() {
    return true; // TODO
}

void ActuatorsControl::enableActuators() {
    for (int i = 0; i < 4; i++) {
        arduino->enableStepper(i);
    }
    arduino->enableServos();
    drive->enable();
}

void ActuatorsControl::disableActuators() {
    arduino->stopMotorDC();
    for (int i = 0; i < 4; i++) {
        arduino->disableStepper(i);
    }
    setPumpState(false, 0);
    arduino->disableServos();
    drive->disable();
}

// ------------------------------------------------------
//                   FACE PALM CONTROL
// ------------------------------------------------------

bool Wall::wDrop() {
    //actuator->setElectroState(true, idx) TODO add electro use
    stoneCount = 0;
    return actuator->setPumpState(false, pumpIdx[0]);
}

bool Wall::wTake() {
    return actuator->setPumpState(true, pumpIdx[0]);
}

// The stone drop electrovalve shares PUMP_2_NUM with the actuator it drives.
bool Wall::wEnableElectro() {
    return actuator->setElectroState(true, PUMP_2_NUM);
}

bool Wall::wDisableElectro() {
    return actuator->setElectroState(false, PUMP_2_NUM);
}

bool Tower::tDrop(int idx) {
    return actuator->setPumpState(false, pumpIdx[idx]);
}

bool Tower::rotate(bool dir) {
    if (dir)
        return actuator->moveServoAndWait(servo, 180, 50); // TODO :Modifier position et speed
    else
        return actuator->moveServoAndWait(servo, 0, 50); // TODO :Modifier position et speed
}

bool Gate::gDrop() { //Version très expérimentale
    tDrop(0);
    tDrop(1);
    rotate(false);
    tDrop(2);
    return true;
}
