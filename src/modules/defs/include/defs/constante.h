#pragma once
#include "drive_interface.h"

#define LOOP_TIME_MS 20

// Duration of a match (100 s).
#define MATCH_DURATION_MS 100000

// Position used before the opponent has been located (off the table).
const position_t OPPONENT_POS_UNKNOWN = { .x = 3000, .y = 0, .a = 0 };

#define RAD_TO_DEG 57.29577951
#define DEG_TO_RAD 0.01745329252

#define ROBOT_WIDTH 400
#define OPPONENT_ROBOT_RADIUS 200
const double TABLE_BORDER_MARGIN_REDUCTION_MM = 50;
const double GAME_ELEMENT_DEPTH_MM = 110;
const double GAME_ELEMENT_WIDTH_MM = 320;
const double GAME_ELEMENT_MARGIN_MM = 50;
const int GAME_ELEMENT_COST = 1000;
const int OPPONENT_COST = 10000;

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
// Define the nums of the arduino for the PUMPS
#define PUMP_3_NUM           6
#define PUMP_2_NUM           7
#define PUMP_1_NUM           8 //TODO change with push on arduino


#define TABLE_X_MAX 1000
#define TABLE_Y_MAX 1500
#define TABLE_X_MIN -1000
#define TABLE_Y_MIN -1500

const position_t ARUCO_POSITIONS_TABLE[] = {
    position_t{.x = -400, .y = -900, .a = 0},
    position_t{.x = -400, .y = 900, .a = 0},
    position_t{.x = 400, .y = -900, .a = 0},
    position_t{.x = 400, .y = 900, .a = 0}};

// Tableau de positions pour la calibration, orientation vers le code le plus proche
// Valeurs uniquement pour le cote bleu
const position_t ARUCO_CALIB_POSITIONS[] = {
    position_t{.x = -125, .y = 350, .a = 0},
    position_t{.x = -125, .y = 675, .a = 0},
    position_t{.x = -100, .y = 1050, .a = 0},
    position_t{.x = 100, .y = 1050, .a = 0},
    position_t{.x = 550, .y = 600, .a = 0},
    position_t{.x = 550, .y = 300, .a = 0}};
    
const int ARUCO_CALIB_POSITIONS_COUNT = sizeof(ARUCO_CALIB_POSITIONS) / sizeof(position_t);


/******* CONSTANTS FOR THE LEGEND OF CAMELOT *******/

// Blue throne room (starting and arrival area)
const position_t START_POSITION = { .x = 0, .y = 1250, .a = -90 };
const position_t FINAL_POSITION = { .x = 0, .y = 1250, .a = -90 };

// La face de depot pointe vers l'exterieur pour garder le robot cote centre du chateau.
const position_t WALL_POSITION[] = {
    position_t{ .x = -600, .y =  1250, .a = 180 }, // 0
    position_t{ .x = -429, .y =   821, .a =-135 }, // 1
    position_t{ .x =    0, .y =   650, .a = -90 }, // 2
    position_t{ .x =  429, .y =   821, .a = -45 }, // 3
    position_t{ .x =  600, .y =  1250, .a =   0 }, // 4
    position_t{ .x = -600, .y = -1250, .a = 180 }, // 5
    position_t{ .x = -429, .y =  -821, .a = 135 }, // 6
    position_t{ .x =    0, .y =  -650, .a =  90 }, // 7
    position_t{ .x =  429, .y =  -821, .a =  45 }, // 8
    position_t{ .x =  600, .y = -1250, .a =   0 }  // 9
};
const int WALL_ZONE_COUNT = sizeof(WALL_POSITION) / sizeof(position_t);

const position_t TOWER_POSITION[] = {
    position_t{ .x = -600, .y =  1000, .a = 0 }, // 0
    position_t{ .x = -250, .y =   650, .a = 0 }, // 1
    position_t{ .x =  250, .y =   650, .a = 0 }, // 2
    position_t{ .x =  600, .y =  1000, .a = 0 }, // 3
    position_t{ .x = -600, .y = -1000, .a = 0 }, // 4
    position_t{ .x = -250, .y =  -650, .a = 0 }, // 5
    position_t{ .x =  250, .y =  -650, .a = 0 }, // 6
    position_t{ .x =  600, .y = -1000, .a = 0 }  // 7
};
const int TOWER_ZONE_COUNT = sizeof(TOWER_POSITION) / sizeof(position_t);

const position_t QUARRY_POSITION[] = {
    position_t{ .x = -950, .y =  750, .a =   180}, // 0
    position_t{ .x = -950, .y =  250, .a =   180}, // 1
    position_t{ .x = -950, .y = -250, .a =   180}, // 2
    position_t{ .x = -950, .y = -750, .a =   180}, // 3
    position_t{ .x = -500, .y =    0, .a = 270}, // 4
    position_t{ .x =  950, .y =  750, .a =   0}, // 5
    position_t{ .x =  950, .y =  250, .a =   0}, // 6
    position_t{ .x =  950, .y = -250, .a =   0}, // 7
    position_t{ .x =  950, .y = -750, .a =   0}, // 8
    position_t{ .x =  500, .y =    0, .a = 270}  // 9
};
const int QUARRY_ZONE_COUNT = sizeof(QUARRY_POSITION) / sizeof(position_t);