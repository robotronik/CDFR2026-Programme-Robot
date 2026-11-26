#include "actions/RobotControl.hpp"

RobotControl::RobotControl(Arduino* arduino)
    : drive(),
      actuators(arduino, &drive),
      wall1(idxWall1, position_t{-70, 120, 120}, &actuators),
      wall2(idxWall2, position_t{0, 0, 0}, &actuators),
      wall3(idxWall3, position_t{0, 0, 0}, &actuators),
      wall4(idxWall4, position_t{0, 0, 0}, &actuators),
      gate1(idxGate1, position_t{0, 0, 0}, &actuators),
      gate2(idxGate2, position_t{0, 0, 0}, &actuators) {

    wallTable[0] = &wall1;
    wallTable[1] = &wall2;
    wallTable[2] = &wall3;
    wallTable[3] = &wall4;
    wallTable[4] = &gate1;
    wallTable[5] = &gate2;

}

void RobotControl::homeActuators()
{
  actuators.homeActuators();
  for(Wall* wall : wallTable) {
    wall->home();
  }
}