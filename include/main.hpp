#pragma once
#include "i2c/Asserv.hpp"
#include "i2c/Arduino.hpp"
#include "defs/tableState.hpp"
#include "defs/structs.hpp"
#include "defs/constante.h"
#include "lidar/Lidar.hpp"

typedef enum {
    INIT = 0,
    WAITSTART = 3,
    RUN = 4,
    FIN = 5,
    MANUAL = 7
} main_State_t;


//Extern means the variable is defined in main but accessible from other classes
extern main_State_t currentState;
extern main_State_t nextState;

extern TableState tableStatus;
extern drive_interface drive;
extern Arduino arduino;
extern Lidar lidar;

extern bool manual_ctrl;
// Declare a function pointer
extern bool (*manual_currentFunc)();

extern bool ctrl_c_pressed;
extern bool ctrl_z_pressed;