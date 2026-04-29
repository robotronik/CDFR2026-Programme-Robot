#pragma once

#include "drive_interface.h"

class DriveControl : private drive_interface {
    public:
        DriveControl();
        ~DriveControl();

        // Read only
        position_t target;
        position_t position;
        position_t velocity;
        position_t acceleration;
        bool is_enabled;
        bool is_slow_mode;

        // Sets the target position, true when done
        bool drive(position_t pos[], int n, bool is_slow_mode = false, bool complete_stop = true);

        // Updates the motion and target
        void update();

        void enable();
        void disable();
        void setBrakeState(bool brake);

        void setGreenLed(bool status);
        void setRedLed(bool status);

        void setCoordinates(position_t pos);

        void logStatus();

        void stopMotion();

        void reset();
};