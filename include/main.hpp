#pragma once
#include "defs/mainState.hpp"
#include "drive_interface.h"
#include "defs/structs.hpp"

//Extern means the variable is defined in main but accessible from other classes
extern main_State_t currentState;
extern main_State_t nextState;


bool returnToHome();

void opponentInAction(position_t position);

void switchTeamSide(colorTeam_t color);

void switchStrategy(std::string strategy);