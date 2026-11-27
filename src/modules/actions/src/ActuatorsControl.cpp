#include "defs/constante.h"
#include "actions/ActuatorsControl.hpp"
#include "i2c/Arduino.hpp"
#include "navigation/driveControl.h"
#include "navigation/navigation.h"
#include "utils/logger.hpp"
#include <cmath>

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
    int s = -1;

    if (servo != prevServo || target != prevTarget) {
        arduino->moveServoSpeed(servo, target, speed);
        prevServo = servo;
        prevTarget = target;
    }

    if (!arduino->getServo(servo, s)) return false;

    return s == target;
}

// ------------------------------------------------------
//                  PUMP CONTROL
// ------------------------------------------------------

bool ActuatorsControl::setPumpState(bool state, int pumpIdx) {
    return arduino->writeSensor(pumpIdx, state);
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
    return true; // add specific home if needed proper home is done in the different faces
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
    setPumpState(false, PUMP_1_NUM);
    arduino->disableServos();
    drive->disable();
}

// ------------------------------------------------------
//                   Walls CONTROL
// ------------------------------------------------------

bool Wall::home(){
    wDisableElectro();
    actuator->setPumpState(false, idx.pumpIdx);
    return DeployBackGripper(false);
}

bool Wall::wDrop() {
    if (!wEnableElectro()
        || !actuator->setPumpState(false, idx.pumpIdx)) return false;
    voidStone();
    return true;
}

bool Wall::wTake() {
    return wDisableElectro()
        && actuator->setPumpState(true, idx.pumpIdx);
}

bool Wall::wEnableElectro() {
    return actuator->setElectroState(true, idx.electroIdx[0]);
}

bool Wall::wDisableElectro() {
    return actuator->setElectroState(false, idx.electroIdx[0]);
}

bool Wall::DeployBackGripper(bool state) {
    return actuator->moveServoAndWait(idx.servoIdx[0], state ? 0 : 120, 360);
}
// ------------------------------------------------------
//                   Towers CONTROL
// ------------------------------------------------------

bool Tower::home(){
    wDisableElectro();
    actuator->setPumpState(false, idx.pumpIdx);
    return rotate(0);
}

bool Tower::tDrop(int index) {
    actuator->setElectroState(true, idx.electroIdx[index]);
    setStone(index, false);
    if(!hasStone()) actuator->setPumpState(false, idx.pumpIdx);
    return true;
}

bool Tower::rotate(int position) {
    constexpr int ANGLES[3] = {180, // Position 0 : Initial position
                            90,  // Position 1 : Tower Position
                            0};  // Position 2 : Gate Position

    if (position < 0 || position > 2) {
        LOG_ERROR("Invalid tower position: ", position);
        return false;
    }

    return actuator->moveServoAndWait(idx.servoIdx[0], ANGLES[position], 50);
}

// ------------------------------------------------------
//                   Gate CONTROL
// ------------------------------------------------------

bool Gate::home(){
    wDisableElectro();
    actuator->setPumpState(false, idx.pumpIdx);
    bool a = rotate(0);
    bool b = gBlockFlip(0);
    return a && b;
}

bool Gate::gBlockFlip(int position) {
    constexpr int ANGLES[3] = {90, // Position 0 : Initial position
                            60,  // Position 1 : Gate Position
                            20};  // Position 2 : Grall Position

    if (position < 0 || position > 2) {
        LOG_ERROR("Invalid gate block flip position: ", position);
        return false;
    }

    return actuator->moveServoAndWait(idx.servoIdx[1], ANGLES[position], 360);
}

bool Gate::gDrop() {
    switch (dropStep) {
    case 0:
        if (!rotate(1)) return false;
        break;

    case 1:
        if (!tDrop(0) || !tDrop(1)) return false;
        retreatTarget = actuator->getDrive()->position;
        retreatTarget.x -= 150 * std::cos(retreatTarget.a * DEG_TO_RAD);
        retreatTarget.y -= 150 * std::sin(retreatTarget.a * DEG_TO_RAD);
        break;

    case 2:
        if (navigation.goTo(retreatTarget, false, true) != NavReturn::Done) return false;
        break;

    case 3:
        if (!rotate(2)) return false;
        break;

    case 4:
        if (!gBlockFlip(1)) return false;
        break;
    
    case 5:
        if (navigation.goTo(retreatTarget, false, true) != NavReturn::Done) return false;
        break;

    case 6:
        if (!tDrop(2)) return false;
        break;

    case 7:
        if (!rotate(0)) return false;
        break;
    }

    if (++dropStep < 7) return false;
    dropStep = 0;
    return true;
}
