#include "drive_interface.h"

drive_interface::drive_interface(){}

uint8_t drive_interface::get_version()
{
    uint8_t version;
    //i2c_read(I2C_ADDRESS, CMD_GET_VERSION, &version, sizeof(version));
    return version;
}
void drive_interface::set_green_led(bool status)
{
    //i2c_write(I2C_ADDRESS, CMD_SET_GREEN_LED, (uint8_t*)&status, sizeof(status));
}
void drive_interface::set_red_led(bool status)
{
    //i2c_write(I2C_ADDRESS, CMD_SET_RED_LED, (uint8_t*)&status, sizeof(status));
}
packed_motion_t drive_interface::get_motion()
{
    packed_motion_t motion;
    //i2c_read(I2C_ADDRESS, CMD_GET_MOTION, (uint8_t*)&motion, sizeof(motion));
    return motion;
}
void drive_interface::set_coordinates(packed_position_t pos)
{
    //i2c_write(I2C_ADDRESS, CMD_SET_COORDINATES, (uint8_t*)&pos, sizeof(pos));
}
packed_position_t drive_interface::get_target()
{
    packed_position_t target;
    //i2c_read(I2C_ADDRESS, CMD_GET_TARGET, (uint8_t*)&target, sizeof(target));
    return target;
}
void drive_interface::set_target(packed_position_t pos)
{
    //i2c_write(I2C_ADDRESS, CMD_SET_TARGET, (uint8_t*)&pos, sizeof(pos));
}
void drive_interface::disable()
{
    //i2c_write(I2C_ADDRESS, CMD_DISABLE, nullptr, 0);
}
void drive_interface::enable()
{
    //i2c_write(I2C_ADDRESS, CMD_ENABLE, nullptr, 0);
}
packed_motor_t drive_interface::get_current()
{
    packed_motor_t current;
    //i2c_read(I2C_ADDRESS, CMD_GET_CURRENT, (uint8_t*)&current, sizeof(current));
    return current;
}
packed_motor_t drive_interface::get_speed()
{
    packed_motor_t speed;
    //i2c_read(I2C_ADDRESS, CMD_GET_SPEED, (uint8_t*)&speed, sizeof(speed));
    return speed;
}
void drive_interface::set_brake_state(bool enable)
{
    //i2c_write(I2C_ADDRESS, CMD_SET_BRAKE_STATE, (uint8_t*)&enable, sizeof(enable));
}
void drive_interface::set_max_torque(double current)
{
    //i2c_write(I2C_ADDRESS, CMD_SET_MAX_TORQUE, (uint8_t*)&current, sizeof(current));
}
status_t drive_interface::get_status()
{
    status_t status;
    //i2c_read(I2C_ADDRESS, CMD_GET_STATUS, (uint8_t*)&status, sizeof(status));
    return status;
}