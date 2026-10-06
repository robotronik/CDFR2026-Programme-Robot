#include "i2c/Arduino.hpp"
#include "navigation/driveControl.h"

class ActuatorsControl {
    public:
        ActuatorsControl(Arduino* arduino, DriveControl* drive);
        bool moveServoAndWait(int servo, int target, int speed);
        bool moveColumnsElevator(int level);
        bool homeActuators();
        void enableActuators();
        void disableActuators();

    private:
        Arduino* arduino;
        DriveControl* drive;
};