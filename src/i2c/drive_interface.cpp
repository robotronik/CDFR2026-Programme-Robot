#include "drive_interface.h"
#include "i2c/i2c.hpp"
#include <unistd.h>				//Needed for I2C port

int i2cFile = -1;

drive_interface::drive_interface(){
#ifndef __CROSS_COMPILE_ARM__
    i2cFile = -1;
#else
    i2cFile = I2COpenSlave(DRIVE_I2C_ADRESS);
#endif
}

drive_interface::~drive_interface(){
    close(i2cFile);
}

uint8_t drive_interface::get_version()
{
    if (i2cFile == -1)
        return DRIVE_I2C_VERSION;
    uint8_t version;
    I2cReceiveData(i2cFile, CMD_GET_VERSION, (uint8_t*)&version, sizeof(version));
    return version;
}
void drive_interface::set_green_led(bool status)
{
    I2cSendData(i2cFile, CMD_SET_GREEN_LED, (uint8_t*)&status, sizeof(status));
}
void drive_interface::set_red_led(bool status)
{
    I2cSendData(i2cFile, CMD_SET_RED_LED, (uint8_t*)&status, sizeof(status));
}
packed_motion_t drive_interface::get_motion()
{
    packed_motion_t motion;
    I2cReceiveData(i2cFile, CMD_GET_MOTION, (uint8_t*)&motion, sizeof(motion));
    return motion;
}
void drive_interface::set_coordinates(packed_position_t pos)
{
    I2cSendData(i2cFile, CMD_SET_COORDINATES, (uint8_t*)&pos, sizeof(pos));
}
packed_position_t drive_interface::get_target()
{
    packed_position_t target;
    I2cReceiveData(i2cFile, CMD_GET_TARGET, (uint8_t*)&target, sizeof(target));
    return target;
}
void drive_interface::set_target(packed_position_t pos)
{
    I2cSendData(i2cFile, CMD_SET_TARGET, (uint8_t*)&pos, sizeof(pos));
}
void drive_interface::disable()
{
    I2cSendData(i2cFile, CMD_DISABLE, nullptr, 0);
}
void drive_interface::enable()
{
    I2cSendData(i2cFile, CMD_ENABLE, nullptr, 0);
}
void drive_interface::calibrate_otos()
{
    I2cSendData(i2cFile, CMD_CALIBRATE_OTOS, nullptr, 0);
}
packed_motor_t drive_interface::get_current()
{
    packed_motor_t current;
    I2cReceiveData(i2cFile, CMD_GET_CURRENT, (uint8_t*)&current, sizeof(current));
    return current;
}
packed_motor_t drive_interface::get_speed()
{
    packed_motor_t speed;
    I2cReceiveData(i2cFile, CMD_GET_SPEED, (uint8_t*)&speed, sizeof(speed));
    return speed;
}
void drive_interface::set_brake_state(bool enable)
{
    I2cSendData(i2cFile, CMD_SET_BRAKE_STATE, (uint8_t*)&enable, sizeof(enable));
}
void drive_interface::set_max_torque(double current)
{
    I2cSendData(i2cFile, CMD_SET_MAX_TORQUE, (uint8_t*)&current, sizeof(current));
}
status_t drive_interface::get_status()
{
    status_t status;
    I2cReceiveData(i2cFile, CMD_GET_STATUS, (uint8_t*)&status, sizeof(status));
    return status;
}

void drive_interface::set_linear_scalar(float scalar)
{
    I2cSendData(i2cFile, CMD_SET_LINEAR_SCALAR, (uint8_t*)&scalar, sizeof(scalar));
}

float drive_interface::get_linear_scalar()
{
    float scalar = 1.0f;
    I2cReceiveData(i2cFile, CMD_GET_LINEAR_SCALAR, (uint8_t*)&scalar, sizeof(scalar));
    return scalar;
}

void drive_interface::set_angular_scalar(float scalar)
{
    I2cSendData(i2cFile, CMD_SET_ANGULAR_SCALAR, (uint8_t*)&scalar, sizeof(scalar));
}

float drive_interface::get_angular_scalar()
{
    float scalar = 1.0f;
    I2cReceiveData(i2cFile, CMD_GET_ANGULAR_SCALAR, (uint8_t*)&scalar, sizeof(scalar));
    return scalar;
}

void drive_interface::set_offset(position_t offset)
{
    packed_position_t p = convertPositionToPacked(offset);
    I2cSendData(i2cFile, CMD_SET_OFFSET, (uint8_t*)&p, sizeof(p));
}

position_t drive_interface::get_offset()
{
    packed_position_t p;
    I2cReceiveData(i2cFile, CMD_GET_OFFSET, (uint8_t*)&p, sizeof(p));
    return convertPackedToPosition(p);
}