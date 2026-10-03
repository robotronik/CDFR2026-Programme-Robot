#include "actions/SensorControl.hpp"
#include "defs/constante.h"

SensorControl::SensorControl(Arduino* arduino) : arduino(arduino) {}

// ------------------------------------------------------
//                    INPUT SENSOR
// ------------------------------------------------------

// Returns true if button sensor was high for the last N calls
bool SensorControl::readButtonSensor(){
    static int count = 0;
    bool state;
    if (!arduino->readSensor(BUTTON_SENSOR_NUM, state)) return false;
    if (state)
        count++;
    else
        count = 0;
    return (count >= 5);
}

// Returns true if the latch sensor is disconnected
bool SensorControl::readLatchSensor(){
    static int count = 0;
    static bool prev_state = false;
    bool state;
    if (!arduino->readSensor(LATCH_SENSOR_NUM, state)) return prev_state;
    if (!state)
        count++;
    else
        count = 0;    
    prev_state = state;
    return (count >= 5);
}

bool SensorControl::readLimitSwitchBottom(){
    bool state;
    if (!arduino->readSensor(LS_BOTTOM_NUM, state)) return false;
    return state;
}

bool SensorControl::readLimitSwitchTop(){
    bool state;
    if (!arduino->readSensor(LS_TOP_NUM, state)) return false;
    return state;
}