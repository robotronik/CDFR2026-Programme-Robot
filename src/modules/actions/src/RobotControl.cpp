#include "actions/RobotControl.hpp"

RobotControl::RobotControl(Arduino* arduino)
    : drive(),
      actuators(arduino, &drive),
      wall1(wall1Tab, position_t{-70, 120, 120}, &actuators),
      wall2(wall2Tab, position_t{0, 0, 0}, &actuators),
      wall3(wall3Tab, position_t{0, 0, 0}, &actuators),
      wall4(wall4Tab, position_t{0, 0, 0}, &actuators),
      gate1(0, 0, gate1Tab, position_t{0, 0, 0}, &actuators),
      gate2(0, 0, gate2Tab, position_t{0, 0, 0}, &actuators)
{
}
