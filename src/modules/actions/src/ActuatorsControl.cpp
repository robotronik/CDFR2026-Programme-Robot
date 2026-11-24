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
//                   Walls CONTROL
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

bool Wall::DeployBackGripper(bool state) {
    return actuator->moveServoAndWait(SERVO_CLAW_CLOSE_1, state ? 0 : 120, 360);
}
// ------------------------------------------------------
//                   Towers CONTROL
// ------------------------------------------------------

bool Tower::tDrop(int idx) {
    return actuator->setPumpState(false, pumpIdx[idx]);
}

// ActuatorsControl.cpp
bool Tower::rotate(int position) {
    constexpr int ANGLES[3] = {180, // Position 0 : Initial position
                            90,  // Position 1 : Tower Position
                            0};  // Position 2 : Gate Position

    if (position < 0 || position > 2) {
        LOG_ERROR("Invalid tower position: ", position);
        return false;
    }

    return actuator->moveServoAndWait(servo, ANGLES[position], 50);
}

// ------------------------------------------------------
//                   Gate CONTROL
// ------------------------------------------------------

bool Gate::gBlockFlip(int position) {
    constexpr int ANGLES[3] = {175, // Position 0 : Initial position
                            90,  // Position 1 : Gate Position
                            40};  // Position 2 : Grall Position

    if (position < 0 || position > 2) {
        LOG_ERROR("Invalid gate block flip position: ", position);
        return false;
    }

    return actuator->moveServoAndWait(servoD, ANGLES[position], 360);
}

bool Gate::gDrop(DriveControl* drive) {
    switch (dropStep) {
    case 0:
        if (!rotate(1)) return false;
        break;

    case 1:
        if (!tDrop(0) || !tDrop(1)) return false;
        retreatTarget = drive->position;
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
