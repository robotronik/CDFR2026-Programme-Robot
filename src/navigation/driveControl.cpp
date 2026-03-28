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
    is_slow_mode = false;
    drive_interface::disable();
    drive_interface::set_target(convertPositionToPacked(target));
    drive_interface::set_coordinates(convertPositionToPacked(position));
    drive_interface::set_brake_state(false);
    drive_interface::set_max_torque(10.0);
}

// Returns true if done or point reached
bool DriveControl::drive(position_t pos[], int n, bool slow_mode, bool complete_stop) {
    if (!is_enabled){
        LOG_WARNING("Not enabled");
        return false;
    }
    if (n <= 0){
        LOG_WARNING("No position given");
        return false;
    }
    is_slow_mode = slow_mode;

    // Calculate the target point along the path
    double looking_distance = 180.0; // Radius (mm)
    
    position_t pos_target;
    double total_distance = position_distance(position, pos[0]);
    int i = 0;
    while (total_distance < looking_distance && i < n - 1) {
        // Calculate the distance to the next position
        total_distance += position_distance(pos[i], pos[i + 1]);
        i++;
    }
    // Use the position at the looking distance
    pos_target = pos[i];

    // Calculate position_speed with acceleration, top speed, and deceleration
    // Angular motion
    double angle_acceleration = (is_slow_mode ? 40.0 : 150.0); // deg/s
    double angle_top_speed   = (is_slow_mode ? 350.0 : 600.0); // deg/s

    double current_angular_velocity = fabs(velocity.a); // deg/s
    double angle_speed;
    
    double error_heading = pos[n-1].a - position.a;
    while (error_heading > 180.0) error_heading -= 360.0;
    while (error_heading < -180.0) error_heading += 360.0;
    
    angle_speed = MIN(current_angular_velocity + angle_acceleration, angle_top_speed);

    const double kP_ang = 10.0;  // Gain for angular speed (deg/s per deg error) (Defined in drive)
    pos_target.a = position.a + MIN(MAX(error_heading, -angle_speed/kP_ang), angle_speed/kP_ang);

    // Linear motion
    double position_acceleration = (is_slow_mode ? 80.0 : 250.0); // mm/s²
    double position_top_speed    =  (is_slow_mode ? 400.0 : 2000.0); // mm/s
    double current_linear_velocity = position_length(velocity); // mm/s
    double position_speed; // mm/s

    position_speed = MIN(current_linear_velocity + position_acceleration, position_top_speed);
    
    // Distance to final target
    double distance_to_target = position_distance(position, pos[n - 1]);

    position_t vec;
    vec.x = pos_target.x - position.x;
    vec.y = pos_target.y - position.y;
    position_normalize(vec);
    const double kP_lin = 5.0;   // Gain for linear speed (mm/s per mm error) (Defined in drive)
    vec.x *= position_speed / kP_lin;
    vec.y *= position_speed / kP_lin;
    if (distance_to_target > position_length(vec) && distance_to_target > 30.0) {
        pos_target.x = position.x + vec.x;
        pos_target.y = position.y + vec.y;
    }
    drive_interface::set_target(convertPositionToPacked(pos_target));

    bool is_done_pos, is_done_ang;
    if (complete_stop){
        is_done_pos = distance_to_target < 5.0 && fabs(velocity.x) < 25.0 && fabs(velocity.y) < 25.0;
        is_done_ang = fabs(error_heading) < 1.0 && fabs(velocity.a) < 4.0;
    }else{
        is_done_pos = distance_to_target < 20.0 && fabs(velocity.x) < 200.0 && fabs(velocity.y) < 200.0;
        is_done_ang = fabs(error_heading) < 2.0 && fabs(velocity.a) < 40.0;
    }
        
    LOG_DEBUG("Pos err: ", distance_to_target, "mm, Vel x: ", velocity.x, "mm/s, Vel y: ", velocity.y, "mm/s, Ang err: ", error_heading, "deg, Ang vel: ", fabs(velocity.a), "deg/s");
    //LOG_DEBUG("Current speed : ", position_length(velocity), "mm/s, Target speed: ", position_speed, "mm/s");
    //LOG_DEBUG("Current speed : ", fabs(velocity.a), "deg/s, Target speed: ", angle_speed, "deg/s");
    target = pos_target; //Update Target
    if (is_done_pos && is_done_ang){
        if (complete_stop){ // If came to a complete stop, set the robot's target to its actual position so it doesn't move more
            drive_interface::set_target(convertPositionToPacked(position));
            target = position; //Update Target
        }
        return true;
    }    
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