#include "navigation/driveControl.h"
#include "utils/logger.hpp"
#include "defs/structs.hpp"
#include <math.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

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
    is_slow_mode = true;
    drive_interface::disable();
    drive_interface::set_target(convertPositionToPacked(target));
    drive_interface::set_coordinates(convertPositionToPacked(position));
    drive_interface::set_brake_state(false);
    drive_interface::set_max_torque(10.0);
}

// Returns true if done or point reached
bool DriveControl::drive(position_t pos[], int n) {
    if (!is_enabled){
        LOG_WARNING("Not enabled");
        return false;
    }
    if (n <= 0){
        LOG_WARNING("No position given");
        return false;
    }

    // Calculate the target point along the path
    position_t pos_target;

    // Calculate a distance along the path
    double min_looking_distance = is_slow_mode ? 25.0 : 25.0; // mm
    double looking_distance_accel = is_slow_mode ? 0.1 : 0.2; // s
    double looking_distance_decel = 0.1; // s
    double looking_distance = position_length(velocity) * looking_distance_accel + min_looking_distance;
    
    double min_looking_angle = is_slow_mode ? 4.0 : 4.0; // degrees
    double looking_angle_accel = is_slow_mode ? 0.35 : 0.5; // s
    double looking_angle_decel = 0.3; // s
    double looking_angle = fabs(velocity.a) * looking_angle_accel + min_looking_angle;

    
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

    double error_heading = pos_target.a - from.a;
    while (error_heading > 180.0) error_heading -= 360.0;
    while (error_heading < -180.0) error_heading += 360.0;
    if (fabs(error_heading) < looking_angle){
        looking_angle -= fabs(velocity.a) * looking_angle_decel; // Reduce a bit the angle
        if (looking_angle < min_looking_angle)
            looking_angle = min_looking_angle;
    }
    pos_target.a = position.a + MIN(MAX(error_heading, -looking_angle), looking_angle);
    // Use the angle to the target

    if (total_distance < looking_distance){
        looking_distance -= position_length(velocity) * looking_distance_decel; // Reduce a bit the distance
        if (looking_distance < min_looking_distance)
            looking_distance = min_looking_distance;
    }

    if (total_distance > looking_distance){
        double resulting_displ = looking_distance - total_distance + position_distance(from, pos_target);
        position_t displacement = position_vector(from, pos_target);
        position_normalize(displacement);
        displacement.x *= resulting_displ;
        displacement.y *= resulting_displ;

        from.x += displacement.x;
        from.y += displacement.y;
        from.a = pos_target.a; // Use the last angle in the path
        drive_interface::set_target(convertPositionToPacked(from));
    }
    else{
        drive_interface::set_target(convertPositionToPacked(pos_target));
    }

    bool is_done_pos = position_distance(position, pos[n - 1]) < 8.0 && fabs(velocity.x) < 1.0 && fabs(velocity.y) < 1.0;
    bool is_done_ang = fabs(error_heading) < 2.0 && fabs(velocity.a) < 5.0;

    if (is_done_pos && is_done_ang && n == 1){
        return true;
    }

    bool has_passed_sub_target = position_distance(position, pos[0]) < looking_distance;
    if (has_passed_sub_target && n > 1)
        return true;
    
    return false; // Not done
}

void DriveControl::update() {
    // Get motion and target
    packed_motion_t mot = drive_interface::get_motion();
    position = convertPackedToPosition(mot.pos);
    velocity = convertPackedToPosition(mot.vel);
    target = convertPackedToPosition(drive_interface::get_target());
}

void DriveControl::setCoordinates(position_t pos) {
    position = pos;
    velocity = {0.0, 0.0, 0.0};
    acceleration = {0.0, 0.0, 0.0};
    drive_interface::set_coordinates(convertPositionToPacked(pos));
    drive_interface::set_target(convertPositionToPacked(pos));
}

void DriveControl::enable(){
    is_enabled = true;
    setCoordinates(position);
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