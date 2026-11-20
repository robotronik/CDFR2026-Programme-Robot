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

enum class ColorTeam {
    None,
    Blue,
    Yellow
};

typedef struct 
{
    double angle;
    double dist;
    double    x;
    double    y;
    bool   onTable;
} lidarAnalize_t;

enum class ConstructionType
{
    Unconstructed=0,
    Wall=8,
    Tower=6,
    Gate=20,
};

typedef struct
{
    int idx; //index of wallZone
    position_t pos; // position of wallZone
    int nb; // nb of stones in wallZone
    ConstructionType bestConstruction; //Best construction in zone
    ColorTeam colorTeam; // Color of the team that owns the zone

} wallZone_t;

typedef struct
{
    int idx; //index of towerZone
    position_t pos; // position of towerZone
    int nb; // nb of stones in towerZone
    ConstructionType bestConstruction; //Best construction in zone
    ColorTeam colorTeam; // Color of the team that owns the zone

} towerZone_t;

typedef struct
{
    int idx; //index of quarry
    position_t pos; // position of quarry
    int nb; // nb of stones in quarry
    float trustFactor;

} quarry_t;

// Define serialization for lidarAnalize_t
void to_json(json& j, const lidarAnalize_t& p);