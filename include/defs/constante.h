#pragma once
#include "defs/structs.hpp"
#define SIZEDATALIDAR 15000

#define LOOP_TIME_MS 20

#define DISTANCESTOP   500

#define RAD_TO_DEG 57.29577951
#define DEG_TO_RAD 0.01745329252

#define ROBOT_WIDTH 400
#define OPPONENT_ROBOT_RADIUS 250

// TODO: is it deprecated value ?
const int STOCK_WIDTH_MM = 100;
const int STOCK_HEIGHT_MM = 400;

// Define the nums of the arduino for the STEPPERS
#define STEPPER_NUM_1        1
#define STEPPER_NUM_2        2
#define STEPPER_NUM_3        3
#define STEPPER_NUM_4        4

// Define the nums of the arduino for the SERVOS
#define SERVO_CLAW_CLOSE_1   1
#define SERVO_SPIN_1         2
#define SERVO_SPIN_2         3
#define SERVO_SPIN_3         4
#define SERVO_SPIN_4         5
#define SERVO_NUM_6          6
#define SERVO_NUM_7          7

// Define the nums of the arduino for the SENSORS
#define BUTTON_SENSOR_NUM    1
#define LATCH_SENSOR_NUM     2
#define SENSOR_NUM_3         3
#define LS_TOP_NUM           4
#define LS_BOTTOM_NUM        5
#define SENSOR_NUM_6         6
#define SENSOR_NUM_7         7
#define SENSOR_NUM_8         8

#define OFFSET_STOCK 250 // Offset to be in the middle of the stock, could be changed if we want to take the stock from the top or the bottom
const int STOCK_COUNT = 8;
const int DROPZONE_COUNT = 10;

// Define the dimensions of the stocks
#define STOCKS_WIDTH 200
#define STOCKS_LENGTH 250

// Define the positions of the stocks 
const position_t STOCK_POSITIONS_TABLE[] = {position_t{.x = -200, .y = 1300, .a = 90}, \
    position_t{.x = 600, .y = 1300, .a = 90},\
    position_t{.x = 800, .y = 400, .a = 0}, \
    position_t{.x = 200, .y = 350, .a = 0}, \

    position_t{.x = -200, .y = -1300, .a = 90}, \
    position_t{.x = 600, .y = -1300, .a = 90}, \
    position_t{.x = 800, .y = -400, .a = 0}, \
    position_t{.x = 200, .y = -350, .a = 0}};
    
    
// The stock table is ordered following symetry of the table, could be ordered by proximity

const position_t STOCK_OFFSETS[] = {
    { 0, OFFSET_STOCK, -90},  //0 : Offset droite
    {0, - OFFSET_STOCK, 90},  //1 : Offset gauche
    { OFFSET_STOCK,0, 0 },  //2 : Offset bas
    { - OFFSET_STOCK,0, 180},  //3 : Offset haut
}; 

const int STOCK_OFFSET_MAPPING[10][2] = {
    {1,-1}, // Stock 0 utilise les offsets 1
    {1,-1}, // Stock 1 utilise les offsets 3
    {3,-1}, // Stock 2 utilise les offsets 7
    {2, 3}, // Stock 3 utilise les offsets 1 et 3

    {0, -1}, // Stock 4 utilise les offsets 0,1,2 et 3
    {0,-1}, // Stock 5 utilise les offsets 2
    {3,-1}, // Stock 6 utilise les offsets 4
    {2,3}, // Stock 7 utilise les offsets 5
};

// Define the dimensions of the Dropzone
#define DROPZONE_WIDTH 200
#define DROPZONE_LENGTH 200

//Define the positions of the Dropzone
const position_t DROPZONE_POSITIONS_TABLE[] = {
    position_t{.x = 200, .y = 1400}, \
    position_t{.x = 900, .y = 800}, \
    position_t{.x = 200, .y = 700}, \
    position_t{.x = -450, .y = 250}, \
    position_t{.x = 200, .y = 0}, \

    position_t{.x = 200, .y = -1400}, \
    position_t{.x = 900, .y = -800}, \
    position_t{.x = 200, .y = -700}, \
    position_t{.x = -450, .y = -250}, \
    position_t{.x = 900, .y = 0}}; \
    // The dropzone table is ordered following symetry of the table, could be ordered by proximity
