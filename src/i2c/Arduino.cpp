#include "i2c/Arduino.hpp"
#include "utils/logger.hpp"
#include "protocol.h"
#include "i2c/i2c.hpp"

#define ARDUINO_SLAVE_ADDRESS 100

Arduino::Arduino(){
    // Check if the device has the same protocol version
#ifndef __CROSS_COMPILE_ARM__
    i2cFile = -1;
#else
    i2cFile = I2COpenSlave(ARDUINO_SLAVE_ADDRESS);
#endif
    if (i2cFile == -1) return; // Emulation
    uint8_t version;
    uint8_t message[] = {0};
    if (I2cSendBlockReceiveData(i2cFile, CMD_GET_VERSION, message, 1, &version, 1) != 0) {
        LOG_ERROR("Failed to read protocol version");
        return;
    }
    if (version != API_VERSION) {
        LOG_ERROR("Protocol version mismatch, expected ", API_VERSION, " but got ", version);
        return;
    } 
    LOG_GREEN_INFO("Protocol version ", version, " is compatible");
}

void Arduino::setServoPower(bool power) {
    uint8_t message [1];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, power);
    I2cSendData(i2cFile, CMD_POWER_SERVOS, message, 1);
}

void Arduino::enableServos() {
    LOG_DEBUG("Enable Servos");
    setServoPower(true);
}


void Arduino::disableServos() {
    LOG_DEBUG("Disable Servos");
    setServoPower(false);
}

// [0;180]
void Arduino::moveServo(int ServoID, int position) {
    LOG_DEBUG("Move servo #", ServoID, " to ", position);
    if (position < 0 || position > 270) {
        LOG_ERROR("Servo position out of range");
        return;
    }
    uint8_t message [5];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, ServoID);
    WriteUInt16(&ptr, position);
    WriteUInt16(&ptr, 0); // Speed (instant)
    I2cSendData(i2cFile, CMD_MOVE_SERVO, message, 5);
}

// Speed in degs/s
void Arduino::moveServoSpeed(int ServoID, int position, int speed) {
    LOG_DEBUG("Move servo #", ServoID, " to ", position, " with speed ", speed);
    if (position < 0 || position > 270) {
        LOG_ERROR("Servo position out of range");
        return;
    }
    uint8_t message [5];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, ServoID);
    WriteUInt16(&ptr, position);
    WriteUInt16(&ptr, speed);
    I2cSendData(i2cFile, CMD_MOVE_SERVO, message, 5);
}

bool Arduino::getServo(int ServoID, int& position){
    //LOG_DEBUG("Get servo #", ServoID);
    if (i2cFile == -1) {position = 0; return true;}; // Emulation
    // position is a int16_t
    uint8_t data[2];
    if (I2cSendBlockReceiveData(i2cFile, CMD_GET_SERVO, (uint8_t*)&ServoID, 1, data, 2))
        return false;
    uint8_t* ptr = data;
    position = ReadInt16(&ptr);
    // LOG_DEBUG("Servo #", ServoID, " is at ", position);
    return true;
}

bool Arduino::readSensor(int SensorID, bool& value){
    if (i2cFile == -1) {value = false; return true;}; // Emulation
    uint8_t data;
    uint8_t message[1];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, SensorID);
    if (I2cSendBlockReceiveData(i2cFile, CMD_READ_SENSOR, message, 1, &data, 1)){
        LOG_WARNING("Couldn't read sensor");
        return false;
    }
    value = data;
    return true;
}

void Arduino::enableStepper(int StepperID) {
    LOG_DEBUG("Enable Stepper #", StepperID);
    I2cSendData(i2cFile, CMD_ENABLE_STEPPER, (uint8_t*)&StepperID, 1);
}

void Arduino::disableStepper(int StepperID) {
    LOG_DEBUG("Disable Stepper #", StepperID);
    I2cSendData(i2cFile, CMD_DISABLE_STEPPER, (uint8_t*)&StepperID, 1);
}

void Arduino::moveStepper(int32_t absPosition, int StepperID) {
    //LOG_DEBUG("Move Stepper #", StepperID, " to ", absPosition);
    uint8_t message [5];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, StepperID);
    WriteInt32(&ptr, absPosition);
    I2cSendData(i2cFile, CMD_MOVE_STEPPER, message, 5);
}

void Arduino::setStepperSpeed(int StepperID, int speed) {
    //LOG_DEBUG("Set Stepper #", StepperID, " speed to ", speed);
    uint8_t message[5];
    uint8_t* ptr = message;
    WriteUInt8(&ptr, StepperID);
    WriteInt32(&ptr, speed);
    I2cSendData(i2cFile, CMD_SET_STEPPER_SPEED, message, 5);
}

void Arduino::setStepper(int32_t absPosition, int StepperID){
    LOG_DEBUG("Set Stepper #", StepperID, " to ", absPosition);
    uint8_t message [5];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, StepperID);
    WriteInt32(&ptr, absPosition);
    I2cSendData(i2cFile, CMD_SET_STEPPER, message, 5);
}

bool Arduino::getStepper(int32_t& absPosition, int StepperID){
    //LOG_DEBUG("Get Stepper #", StepperID);
    if (i2cFile == -1) {absPosition = 0; return true;} // Emulation
    uint8_t data[4];
    if (I2cSendBlockReceiveData(i2cFile, CMD_GET_STEPPER, (uint8_t*)&StepperID, 1, data, 4))
        return false;
    uint8_t* ptr = data;
    absPosition = ReadInt32(&ptr);
    LOG_DEBUG("Stepper #", StepperID, " is at ", absPosition);
    return true;
}

void Arduino::RGB(int LED_ID, uint8_t mode, uint8_t r, uint8_t g, uint8_t b){
    if (i2cFile == -1) return; // Emulation
    uint8_t message [5];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, LED_ID);
    WriteUInt8(&ptr, mode);
    if (mode != 2){
        WriteUInt8(&ptr, r);
        WriteUInt8(&ptr, g);
        WriteUInt8(&ptr, b);
        I2cSendData(i2cFile, CMD_RGB_LED, message, 5);
    }
    else 
        I2cSendData(i2cFile, CMD_RGB_LED, message, 2); // Rainbow mode
}

void Arduino::RGB_Solid(uint8_t R, uint8_t G, uint8_t B, int LED_ID){
    LOG_DEBUG("Set RGB LED #", LED_ID, " to solid color (", (int)R, ", ", (int)G, ", ", (int)B, ")");
    RGB(LED_ID, 0, R, G, B);
}

void Arduino::RGB_Blinking(uint8_t R, uint8_t G, uint8_t B, int LED_ID){
    LOG_DEBUG("Set RGB LED #", LED_ID, " to blinking color (", (int)R, ", ", (int)G, ", ", (int)B, ")");
    RGB(LED_ID, 1, R, G, B);
}

void Arduino::RGB_Rainbow(int LED_ID){
    LOG_DEBUG("Set RGB LED #", LED_ID, " to rainbow");
    RGB(LED_ID, 2, 0, 0, 0);
}

void Arduino::SetLidarPWM(uint8_t val){
    if (i2cFile, I2cSendData(i2cFile, CMD_SET_PWM_LIDAR, &val, 1))
        LOG_ERROR("Couldn't set Lidar PWM");
}


void Arduino::moveMotorDC(uint8_t speed, uint8_t holding){
    //LOG_DEBUG("Moving DC Motor ");
    if (i2cFile == -1) return; // Emulation
    uint8_t message [3];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, 1);
    WriteUInt8(&ptr, speed);
    WriteUInt8(&ptr, holding);
    I2cSendData(i2cFile, CMD_MOVE_DC_MOTOR, message, 3);
    LOG_DEBUG("DC Motor moved");
}
void Arduino::stopMotorDC(){
    //LOG_DEBUG("Stopping DC Motor ");
    if (i2cFile == -1) return; // Emulation
    uint8_t message [1];
    uint8_t *ptr = message;
    WriteUInt8(&ptr, 1);
    I2cSendData(i2cFile, CMD_STOP_DC_MOTOR, message, 1);
}