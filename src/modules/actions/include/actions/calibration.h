#ifndef CALIBRATION_H
#define CALIBRATION_H
#include "defs/tableState.hpp"

#include <stdbool.h>
class Robot;

bool calibrate_otos(Robot* robot, position_t startPos, bool skip_calibration = true);

#endif // CALIBRATION_H