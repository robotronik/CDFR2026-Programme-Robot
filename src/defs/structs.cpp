#include "defs/structs.hpp"
#include <math.h>


// Define a function to calculate the angle in rads from and to a position_t
double position_angle(position_t from, position_t to){
    // Calculates the angle between the two points
    return atan2(to.y - from.y, to.x - from.x);
}

// Define a function to calculate the distance from and to a position_t
double position_distance(position_t from, position_t to){
    // Calculates the position between the two points
    return sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
}

double position_length(position_t pos){
    // Calculates the length of the position vector
    return sqrt(pow(pos.x, 2) + pow(pos.y, 2));
}   

position_t position_vector(position_t from, position_t to){
    // Calculates the vector between the two points
    position_t vector;
    vector.x = to.x - from.x;
    vector.y = to.y - from.y;
    vector.a = normalize_angle(position_angle(from, to));
    return vector;
}

void position_normalize(position_t& pos){
    // Normalize the vector
    double norm = position_length(pos);
    pos.x /= norm;
    pos.y /= norm;
    // Normalize the angle
    if (pos.a > 180)
        pos.a -= 360;
    else if (pos.a < -180)
        pos.a += 360;
}
double normalize_angle(double angle) {
    // Normalize the angle to be between -180 and 180 degrees
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

void position_robot_flip(position_t& pos){
    pos.y = -pos.y;
    if (pos.a >= 0)
        pos.a = 180 - pos.a;
    else
        pos.a = -180 - pos.a;
}


//Define serialization for position_t
void to_json(json& j, const position_t& p) {
    j = json{
        {"x", p.x},
        {"y", p.y},
        {"a", p.a}
    };
}

// Define serialization for lidarAnalize_t
void to_json(json& j, const lidarAnalize_t& p) {
    j = json{
        {"A", p.angle},
        {"R", p.dist},
        {"x", p.x},
        {"y", p.y},
        {"onTable", p.onTable}
    };
}