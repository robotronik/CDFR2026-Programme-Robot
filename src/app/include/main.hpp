#pragma once
#include "drive_interface.h"
#include "defs/structs.hpp"


bool returnToHome();

void opponentInAction(position_t position);

void switchTeamSide(ColorTeam color);

void switchStrategy(std::string strategy);

void update_team_strat();