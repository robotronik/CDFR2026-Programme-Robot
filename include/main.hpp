#pragma once
#include "i2c/Arduino.hpp"
#include "defs/mainState.hpp"
#include "defs/tableState.hpp"
#include "lidar/Lidar.hpp"
#include "navigation/driveControl.h"
#include "vision/ArucoCam.hpp"

//Extern means the variable is defined in main but accessible from other classes
extern main_State_t currentState;
extern main_State_t nextState;

extern TableState tableStatus;
extern DriveControl drive;
extern Arduino arduino;
extern Lidar lidar;
extern ArucoCam arucoCam1;

extern bool exit_requested;