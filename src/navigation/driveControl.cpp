#include "navigation/driveControl.h"
#include "utils/logger.hpp"

DriveControl::DriveControl() {
    // TODO Connect I2C
    int version = drive_interface::get_version();

    if (version == I2C_VERSION){
        // GOOD
    }

    reset();
}

void DriveControl::reset() {
    position.x = 0.0;
    position.y = 0.0;
    position.a = 0.0;
    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.a = 0.0;
    acceleration.x = 0.0;
    acceleration.y = 0.0;
    acceleration.a = 0.0;
    is_enabled = false;
    drive_interface::disable();
    drive_interface::set_target(convertPositionToPacked(target));
    drive_interface::set_coordinates(convertPositionToPacked(position));
    drive_interface::set_brake_state(false);
    drive_interface::set_max_torque(10.0);
}

void DriveControl::drive(position_t pos[], int n) {
    // TODO
    target = pos[0];
    drive_interface::set_target(convertPositionToPacked(target));
}

void DriveControl::update() {
    // Get motion and target
    packed_motion_t mot = drive_interface::get_motion();
    position = convertPackedToPosition(mot.pos);
    velocity = convertPackedToPosition(mot.vel);
    acceleration = convertPackedToPosition(mot.acc);
    target = convertPackedToPosition(drive_interface::get_target());
}

void DriveControl::enable(){
    is_enabled = true;
    drive_interface::enable();
}

void DriveControl::disable(){
    is_enabled = false;
    drive_interface::disable();
}

void DriveControl::setGreenLed(bool status){
    drive_interface::set_green_led(status);
}

void DriveControl::setRedLed(bool status){
    drive_interface::set_red_led(status);
}

void DriveControl::logStatus(){
    status_t status = drive_interface::get_status();
    LOG_INFO("Status 1: ", status.is_error1);
    LOG_INFO("Status 2: ", status.is_error2);
}