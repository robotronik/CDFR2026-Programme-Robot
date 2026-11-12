#pragma once
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
position_t position_sum(position_t leftOp, position_t rightOp);
bool position_equals(position_t leftOp, position_t rightOp);

// Define a function to calculate the distance from and to a position_t
double position_distance(position_t from, position_t to);
void position_normalize(position_t& pos);
void position_robot_flip(position_t& pos);
double position_length(position_t pos);
double normalize_angle(double angle);
position_t position_vector(position_t from, position_t to);

typedef struct 
{
    double angle;
    double dist;
    double    x;
    double    y;
    bool   onTable;
} lidarAnalize_t;

typedef enum {
    NONE,
    BLUE,
    YELLOW
} colorTeam_t;

typedef struct {
    int id; // index of quarry
    int nb; // number of rocks in quarry
    position_t pos; // position of quarry
    float trustFactor; // % of trust on this quarry
    // TODO is it a good idea to add position of each rock ?
} quarry_t;

// Define serialization for lidarAnalize_t
void to_json(json& j, const lidarAnalize_t& p);