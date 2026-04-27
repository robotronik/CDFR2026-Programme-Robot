#pragma once

#include "defs/structs.hpp"
#include <utils/json.hpp>
using json = nlohmann::json;

void convertAngularToAxial(lidarAnalize_t* data, int count, position_t position, int narrow);
bool position_opponentV2(lidarAnalize_t* data, int count, position_t robot_pos, position_t &opponent_pos);
bool opponent_collide_lidar(lidarAnalize_t* data, int count, int width, int depth, int margin, double angle);
double distance_2_pts(double d1,double deg1, double d2, double deg2);

int delta_angle(int angle1, int angle2);
double delta_angle_double(double angle1, double angle2);