#pragma once

#include "drive_interface.h"

class DriveControl : public drive_interface {
    public:
        DriveControl();
        ~DriveControl();

        // Read only
        position_t target;
        position_t position;
        position_t velocity;
        position_t acceleration;
        bool is_enabled;

        // Sets the target position, true when done
        bool drive(position_t pos[], int n);

        // Updates the motion and target
        void update();

        void enable();
        void disable();

        void setGreenLed(bool status);
        void setRedLed(bool status);

        void logStatus();

    private:
        bool connected;
        void reset();
};