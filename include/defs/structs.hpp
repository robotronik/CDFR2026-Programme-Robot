#pragma once
#include "utils/utils.h"
#include <utils/json.hpp>
using json = nlohmann::json;

typedef struct 
{
    int  x;
    int  y;
    int  a;
}position_t; 

//Define serialization for position_t
void to_json(json& j, const position_t& p);

// Define a function to calculate the angle in rads from and to a position_t
double position_angle(position_t from, position_t to);

// Define a function to calculate the distance from and to a position_t
double position_distance(position_t from, position_t to);

void position_robot_flip(position_t& pos);

typedef struct {
    float x;
    float y;
    float angle;
} position_float_t;

typedef struct {
    double x;
    double y;
    double angle;
} position_double_t;

//Define serialization for position_t
void to_json(json& j, const position_double_t& p);

typedef struct 
{
    double angle;
    double dist;
    int    x;
    int    y;
    bool   onTable;
}lidarAnalize_t;

// Define serialization for lidarAnalize_t
void to_json(json& j, const lidarAnalize_t& p);

typedef struct
{
    colorTeam_t colorTeam;// équipe du robot
    position_t pos; //position du robot
    position_t target;
    double vit_x,vit_y; //TODO vitesse instantannée du robot
    int braking_distance;
    int direction_side;
    int plank_count;
}robot_t;

// Define serialization for robot_t
void to_json(json& j, const robot_t& p);