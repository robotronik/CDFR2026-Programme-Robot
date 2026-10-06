#ifndef CALIBRATION_H
#define CALIBRATION_H
#include "navigation/driveControl.h"
#include "defs/tableState.hpp"

#include <stdbool.h>
bool calibrate_otos(TableState* tableStatus, DriveControl* drive, position_t startPos);

#endif // CALIBRATION_H