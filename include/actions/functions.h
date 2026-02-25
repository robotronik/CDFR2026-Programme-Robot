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
bool rotateBlocks();
bool lowerClaws();
bool raiseClaws();
bool rotateTwoBlocksEnd();
bool rotateTwoBlocks(bool endWithlower);
bool dropBlock();

// Related to stock management
int GetBestDropZone(position_t fromPos);
void setStockAsRemoved(int num);
void setDropzoneAsError(int dropzoneNum);
bool getBestStockPositionOff(int stockNum, position_t fromPos);

// Servo Control
bool closeClaws();
bool openClaws();
bool snapClaws(bool closed);
bool resetSpinClaws();
bool spinAllClaws();
bool spinClaws(bool spin1, bool spin2, bool spin3, bool spin4);

// Stepper Control
bool moveColumnsElevator(int level); // Example

// DC Motor Control
bool moveTribuneElevator();
void stopTribuneElevator();

// Input sensors
bool readButtonSensor();
bool readLatchSensor();
bool readLimitSwitchBottom();
bool readLimitSwitchTop();

#endif // MYFUNCTION_H
