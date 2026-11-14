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

typedef enum {
    NONE,
    BLUE,
    YELLOW
} colorTeam_t;

typedef struct 
{
    double angle;
    double dist;
    double    x;
    double    y;
    bool   onTable;
} lidarAnalize_t;

typedef enum
{
    UNCONSTRUCTED=0,
    WALL=8,
    TOWER=6,
    DOOR=23,
} constructionType_t;

typedef struct
{
    int idx; //index of dropZone
    position_t pos; // position of dropZone
    int nb; // nb of rocks in dropZone
    constructionType_t bestConstruction; //Best construction in zone
    colorTeam_t colorTeam; // Color of the team that owns the zone

} wallZone_t;

typedef struct
{
    int idx; //index of dropZone
    position_t pos; // position of dropZone
    int nb; // nb of rocks in dropZone
    constructionType_t bestConstruction; //Best construction in zone
    colorTeam_t colorTeam; // Color of the team that owns the zone

} towerZone_t;

// Define serialization for lidarAnalize_t
void to_json(json& j, const lidarAnalize_t& p);