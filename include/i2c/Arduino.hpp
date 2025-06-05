#pragma once

#include <cstdint>
#include <iostream>

class Arduino {
   
public:
    Arduino();

    // Functions return true if successfully executed
    void enableServos();
    void disableServos();
    void moveServo(int ServoID, int position);
    void moveServoSpeed(int ServoID, int position, int speed);
    bool getServo(int ServoID, int& position);
    bool readSensor(int SensorID, bool& value);
    void moveStepper(int32_t absPosition, int StepperID);
    void setStepperSpeed(int StepperID, int speed);
    void setStepper(int32_t absPosition, int StepperID);
    bool getStepper(int32_t& absPosition, int StepperID);
    void enableStepper(int StepperID);
    void disableStepper(int StepperID);
    void RGB_Solid(uint8_t R, uint8_t G, uint8_t B, int LED_ID = 1);
    void RGB_Blinking(uint8_t R, uint8_t G, uint8_t B, int LED_ID = 1);
    void RGB_Rainbow(int LED_ID = 1);
    void SetLidarPWM(uint8_t val);
    void moveMotorDC(uint8_t speed, uint8_t holding);
    void stopMotorDC();
private:
    int i2cFile;
    void setServoPower(bool power);
    void RGB(int LED_ID, uint8_t mode, uint8_t r, uint8_t g, uint8_t b);
};