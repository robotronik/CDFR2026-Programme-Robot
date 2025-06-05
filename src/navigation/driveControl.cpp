#include "navigation/driveControl.h"
#include "utils/logger.hpp"
#include "defs/structs.hpp"
#include <math.h>

DriveControl::DriveControl() {
    int version = drive_interface::get_version();

    if (version != DRIVE_I2C_VERSION) {
        LOG_ERROR("Protocol version mismatch, expected ", DRIVE_I2C_VERSION, " but got ", version);
        return;
    } 
    LOG_GREEN_INFO("Protocol version ", version, " is compatible");

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

// Returns true if done
bool DriveControl::drive(position_t pos[], int n) {
    if (!is_enabled){
        LOG_WARNING("Not enabled");
        return false;
    }
    if (n <= 0){
        LOG_WARNING("No position given");
        return false;
    }

    // TODO Check if the robot is moving
    const double max_lin_speed = 1000.0; // mm/s
    const double max_ang_speed = 180.0; // degrees/s
    double linear_speed = position_length(velocity);
    double angular_speed = fabs(velocity.a);

    // Calculate the target point along the path
    position_t pos_target;

    // Calculate a distance along the path
    const double looking_distance = fmin(100.0 + linear_speed, max_lin_speed) / 3.0;
    double total_distance = position_distance(position, pos[0]);
    position_t from = position;
    int i = 0;
    while (total_distance < looking_distance && i < n - 1) {
        // Calculate the distance to the next position
        total_distance += position_distance(pos[i], pos[i + 1]);
        from = pos[i];
        i++;
    }
    // Use the position at the looking distance
    pos_target = pos[i];
    double resulting_displ = looking_distance - total_distance + position_distance(from, pos_target);
    position_t displacement = position_vector(from, pos_target);
    position_normalize(displacement);
    displacement.x *= resulting_displ;
    displacement.y *= resulting_displ;

    from.x += displacement.x;
    from.y += displacement.y;

    from.a = pos[n-1].a; // Use the last angle in the path
    
    drive_interface::set_target(convertPositionToPacked(from));
    return false; // TODO return true if not moving
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