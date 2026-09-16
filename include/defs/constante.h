#pragma once
#include "defs/structs.hpp"
#define SIZEDATALIDAR 15000

#define LOOP_TIME_MS 20

#define DISTANCESTOP   500

#define RAD_TO_DEG 57.29577951
#define DEG_TO_RAD 0.01745329252

#define ROBOT_WIDTH 400
#define OPPONENT_ROBOT_RADIUS 250

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


#define MAX_WIDTH_TABLE 1000
#define MAX_LENGTH_TABLE 1500

#define CALIBRATION_DEPLETION_TIME 2 // Max time between to calibration
#define D_THRESHOLD_LATERAL 500

#define AS_THRESHOLD 200
#define ADVERSARY_THRESH 300

const position_t ARUCO_POSITIONS_TABLE[] = {
    position_t{.x = -400, .y = -900, .a = 0}, \
    position_t{.x = -400, .y = 900, .a = 0}, \
    position_t{.x = 400, .y = -900, .a = 0}, \
    position_t{.x = 400, .y = 900, .a = 0}};

// Tableau de positions pour la calibration, orientation vers le code le plus proche
// Valeurs uniquement pour le cote bleu
const position_t ARUCO_CALIB_POSITIONS[] = {
    position_t{.x = -125, .y = 350, .a = 0}, \
    position_t{.x = -125, .y = 675, .a = 0}, \
    position_t{.x = -100, .y = 1050, .a = 0}, \
    position_t{.x = 100, .y = 1050, .a = 0}, \
    position_t{.x = 550, .y = 600, .a = 0}, \
    position_t{.x = 550, .y = 300, .a = 0}};
    
const int ARUCO_CALIB_POSITIONS_COUNT = sizeof(ARUCO_CALIB_POSITIONS) / sizeof(position_t);

/******* CONSTANT FOR THE LEGEND OF CAMELOT *******/