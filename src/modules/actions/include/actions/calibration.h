#ifndef CALIBRATION_H
#define CALIBRATION_H
#include "defs/tableState.hpp"

#include <stdbool.h>
class RobotControl;
class Cam;

bool calibrate_otos(TableState* tableStatus, RobotControl* robot, Cam* cam, position_t startPos, bool skip_calibration = true);

#endif // CALIBRATION_H