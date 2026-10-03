#ifndef CALIBRATION_H
#define CALIBRATION_H
#include "navigation/driveControl.h"
#include "defs/tableState.hpp"

#include <stdbool.h>
bool calibrate_otos(TableState* tableStatus, DriveControl* drive);

#endif // CALIBRATION_H