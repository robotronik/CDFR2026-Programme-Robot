#ifndef MYFUNCTION_H
#define MYFUNCTION_H

#include "defs/structs.hpp"
#include "main.hpp"

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

// Related to stock management
int GetBestDropZone(position_t fromPos);
void setStockAsRemoved(int num);
void setDropzoneState(int dropzoneNum, TableState::dropzone_state_t state);
void setDropzoneAsError(int dropzoneNum);
int getBestStockPositionOff(int stockNum, position_t fromPos);
position_t getBestDropZonePosition(int dropzoneNum, position_t fromPos);
position_t calculateClosestArucoPosition(position_t currentPos, position_t& outPos);

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
