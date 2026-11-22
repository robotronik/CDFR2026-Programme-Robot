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
    return arduino->setPump(pumpIdx, state);
}

bool ActuatorsControl::setElectroState(bool state, int electroIdx) {
    return arduino->setElectro(electroIdx, state);
}

// ------------------------------------------------------
//                GLOBAL SET/RES CONTROL
// ------------------------------------------------------

// Returns true if actuators are home
bool ActuatorsControl::homeActuators() {
    return true; // TODO
}

void ActuatorsControl::enableActuators() {
    arduino->enableServos();
    drive->enable();
}

void ActuatorsControl::disableActuators() {
    for (int i = 1; i <= PUMP_COUNT; i++) {
        setPumpState(false, i);
    }
    for (int i = 1; i <= EV_COUNT; i++) {
        setElectroState(false, i);
    }
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
