#pragma once
#include "utils/utils.h"
#include "drive_interface.h"
#include <utils/json.hpp>
using json = nlohmann::json;

/* Already defined in drive_interface.h
typedef struct 
{
    double  x, y, a;
}position_t;*/

//Define serialization for position_t
void to_json(json& j, const position_t& p);

// Define a function to calculate the angle in rads from and to a position_t
double position_angle(position_t from, position_t to);

// Define a function to calculate the distance from and to a position_t
double position_distance(position_t from, position_t to);

void position_robot_flip(position_t& pos);

typedef struct 
{
    double angle;
    double dist;
    int    x;
    int    y;
    bool   onTable;
} lidarAnalize_t;

// Define serialization for lidarAnalize_t
void to_json(json& j, const lidarAnalize_t& p);