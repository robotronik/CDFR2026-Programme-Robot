#ifndef MYFUNCTION_H
#define MYFUNCTION_H

#include "defs/structs.hpp"

bool returnToHome();
bool homeActuators();
void enableActuators();
void disableActuators();
bool isRobotInArrivalZone(position_t position);
void opponentInAction(position_t position);
void switchTeamSide(colorTeam_t color);
void switchStrategy(int strategy);


// Basic functions (FSM)
//bool lowerClaws(); function must be define here 


// Servo Control
bool moveServoAndWait(int servo, int target, int speed); // Example


// Stepper Control
bool moveColumnsElevator(int level); // Example

// Input sensors
bool readButtonSensor();
bool readLatchSensor();
bool readLimitSwitchBottom();
bool readLimitSwitchTop();

#endif // MYFUNCTION_H
