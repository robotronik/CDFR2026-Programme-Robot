#include "navigation/driveControl.h"
#include "utils/logger.hpp"

DriveControl::DriveControl() {
    // TODO Connect I2C
    int version = drive_interface::get_version();

    if (version == DRIVE_I2C_VERSION){
        // GOOD
    }

    reset();
}
DriveControl::~DriveControl() {}

void DriveControl::reset() {
    position = {0.0, 0.0, 0.0};
    velocity = {0.0, 0.0, 0.0};
    acceleration = {0.0, 0.0, 0.0};
    is_enabled = false;
    drive_interface::disable();
    drive_interface::set_target(convertPositionToPacked(target));
    drive_interface::set_coordinates(convertPositionToPacked(position));
    drive_interface::set_brake_state(false);
    drive_interface::set_max_torque(10.0);
}

void DriveControl::drive(position_t pos[], int n) {
    if (!is_enabled){
        LOG_WARNING("Not enabled");
        return;
    }
    if (n == 0){
        LOG_WARNING("No position given");
        return;
    }
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
    if (status.is_error1)
        LOG_ERROR("Status 1 !");
    if (status.is_error2)
        LOG_ERROR("Status 2 !");
}