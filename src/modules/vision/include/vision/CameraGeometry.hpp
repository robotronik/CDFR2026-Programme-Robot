#pragma once

// Field size in millimetres; the map and the rectified patch are both gridded
// from FIELD_WIDTH_MM.
constexpr double FIELD_WIDTH_MM = 2000.0;

// Camera mounting, measured on the real robot from the landmark-tag poses
// (tests/data/aruco_loc): height above the table and downward tilt. The field of
// view comes from the calibration instead, so any lens works.
constexpr double CAMERA_HEIGHT_MM = 218.0;
constexpr double CAMERA_PITCH_DEG = 44.5;

namespace vision {

// Camera field pose: x/y in millimetres, z the height above the ground, heading
// in degrees normalized to ]-180, 180].
struct CameraPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0;
};

} // namespace vision
