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
    double looking_distance = is_slow_mode ? 40.0 : 80.0; // Radius (mm)
    
    position_t pos_target;
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

    // Calculate angle_speed with acceleration, top speed, and deceleration
    double angle_acceleration = is_slow_mode ? 15.0 : 150.0; // deg/s
    double angle_top_speed = is_slow_mode ? 100.0 : 800.0; // deg/s
    double angle_deceleration = is_slow_mode ? 150.0 : 600.0; // deg/s²
    
    double current_angular_velocity = fabs(velocity.a); // deg/s
    double angle_speed;
    
    double error_heading = pos_target.a - position.a;
    while (error_heading > 180.0) error_heading -= 360.0;
    while (error_heading < -180.0) error_heading += 360.0;
    
    // Calculate stopping distance: distance = v² / (2 * a)
    double angular_stopping_distance = (current_angular_velocity * current_angular_velocity) / (2.0 * angle_deceleration);
    
    // Accelerate if below top speed, decelerate if close to target
    if (fabs(error_heading) <= angular_stopping_distance) {
        // Deceleration phase - calculate target speed based on remaining distance
        double target_speed = sqrt(2.0 * angle_deceleration * fabs(error_heading));
        angle_speed = MAX(MIN(target_speed, current_angular_velocity), 5.0);
    } else if (current_angular_velocity < angle_top_speed) {
        // Acceleration phase
        angle_speed = MIN(current_angular_velocity + angle_acceleration, angle_top_speed);
    } else {
        // Maintain top speed
        angle_speed = angle_top_speed;
    }

    const double kP_ang = 8.0;  // Gain for angular speed (deg/s per deg error) (Defined in drive)
    pos_target.a = position.a + MIN(MAX(error_heading, -angle_speed/kP_ang), angle_speed/kP_ang);

    // Calculate position_speed with acceleration, top speed, and deceleration
    double position_acceleration = is_slow_mode ? 70.0 : 100.0; // mm/s
    double position_top_speed = is_slow_mode ? 500.0 : 2000.0; // mm/s
    double position_deceleration = is_slow_mode ? 800.0 : 1200.0; // mm/s²
    
    double current_linear_velocity = position_length(velocity); // mm/s
    double position_speed; // mm/s
    
    // Distance to final target
    double distance_to_target = position_distance(position, pos[n - 1]);
    
    // Calculate stopping distance: distance = v² / (2 * a)
    double linear_stopping_distance = (current_linear_velocity * current_linear_velocity) / (2.0 * position_deceleration);
    
    // Accelerate if below top speed, decelerate if close to target
    if (distance_to_target <= linear_stopping_distance) {
        // Deceleration phase - calculate target speed based on remaining distance
        double target_speed = sqrt(2.0 * position_deceleration * distance_to_target);
        position_speed = MAX(MIN(target_speed, current_linear_velocity), 10.0);
        position_speed = MIN(position_speed, current_linear_velocity);
    } else if (current_linear_velocity < position_top_speed) {
        // Acceleration phase
        position_speed = MIN(current_linear_velocity + position_acceleration, position_top_speed);
    } else {
        // Maintain top speed
        position_speed = position_top_speed;
    }

    if (total_distance > looking_distance){
        double resulting_displ = looking_distance - total_distance + position_distance(from, pos_target);
        position_t displacement = position_vector(from, pos_target);
        position_normalize(displacement);
        displacement.x *= resulting_displ;
        displacement.y *= resulting_displ;

        from.x += displacement.x;
        from.y += displacement.y;
        position_t vec;
        vec.x = from.x - position.x;
        vec.y = from.y - position.y;
        position_normalize(vec);
        const double kP_lin = 6.0;   // Gain for linear speed (mm/s per mm error) (Defined in drive)
        vec.x *= position_speed / kP_lin;
        vec.y *= position_speed / kP_lin;
        pos_target.x = position.x + vec.x;
        pos_target.y = position.y + vec.y;
        drive_interface::set_target(convertPositionToPacked(pos_target));
    }
    else{
        drive_interface::set_target(convertPositionToPacked(pos_target));
    }

    bool is_done_pos = position_distance(position, pos[n - 1]) < 5.0 && fabs(velocity.x) < 30.0 && fabs(velocity.y) < 30.0;
    bool is_done_ang = fabs(error_heading) < 1.0 && fabs(velocity.a) < 5.0;
    //LOG_DEBUG("Position error: ", position_distance(position, pos[n - 1]), "mm, Velocity: ", position_length(velocity), "mm/s, Angle error: ", error_heading, "deg, Angular velocity: ", fabs(velocity.a), "deg/s");

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