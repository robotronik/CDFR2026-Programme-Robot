#include "i2c/Arduino.hpp"

class SensorControl {
    public:
        SensorControl(Arduino* arduino);
        bool readButtonSensor();
        bool readLatchSensor();
        bool readLimitSwitchBottom();
        bool readLimitSwitchTop();

    private:
        Arduino* arduino;
};