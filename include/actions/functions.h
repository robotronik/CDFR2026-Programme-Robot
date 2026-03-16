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
bool lowerClaws();
bool raiseClaws();
bool rotateTwoBlocks(bool *order);
bool rotateTwoBlocksDefault();
bool dropBlock();
bool enableCursor(bool enable);


// Servo Control
bool closeClaws();
bool openClaws();
bool snapClaws(bool closed);
bool snapClaws(bool closed, bool small);
bool resetSpinClaws();
bool spinAllClaws();
bool spinClaws(bool spin1, bool spin2, bool spin3, bool spin4);

// Stepper Control
bool moveColumnsElevator(int level); // Example

// Input sensors
bool readButtonSensor();
bool readLatchSensor();
bool readLimitSwitchBottom();
bool readLimitSwitchTop();

#endif // MYFUNCTION_H
