#include "actions/RobotControl.hpp"

RobotControl::RobotControl(Arduino* arduino)
    : drive(),
      actuators(arduino, &drive),
      wall1(wall1Tab, position_t{0, 0, 0}, &actuators),
      wall2(wall2Tab, position_t{0, 0, 0}, &actuators),
      wall3(wall3Tab, position_t{0, 0, 0}, &actuators),
      wall4(wall4Tab, position_t{0, 0, 0}, &actuators),
      door1(0, 0, door1Tab, position_t{0, 0, 0}, &actuators),
      door2(0, 0, door2Tab, position_t{0, 0, 0}, &actuators)
{
}
