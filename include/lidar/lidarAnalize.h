#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "defs/structs.hpp"
#include "math.h"
#include "utils/logger.hpp"
#include <utils/json.hpp>
using json = nlohmann::json;

void convertAngularToAxial(lidarAnalize_t* data, int count, position_t position, int narrow);
void convertAngularToAxialCompensated(lidarAnalize_t* data, int count,
    position_t start, position_t velocity,
    double scan_period, int narrow);
bool position_opponentV2(lidarAnalize_t* data, int count, position_t robot_pos, position_t &opponent_pos);
bool opponent_collide_lidar(lidarAnalize_t* data, int count, int width, int depth, int margin);
double distance_2_pts(double d1,double deg1, double d2, double deg2);

int delta_angle(int angle1, int angle2);
double delta_angle_double(double angle1, double angle2);