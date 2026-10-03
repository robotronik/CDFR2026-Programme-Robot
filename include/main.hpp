#pragma once
#include "drive_interface.h"
#include "defs/structs.hpp"


bool returnToHome();

void opponentInAction(position_t position);

void switchTeamSide(colorTeam_t color);

void switchStrategy(std::string strategy);