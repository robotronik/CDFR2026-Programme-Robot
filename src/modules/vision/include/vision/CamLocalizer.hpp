#pragma once

#define FIELD_WIDTH_MM 2000.0
#define FIELD_HEIGHT_MM 3000.0

// Camera mounting, measured on the real robot from the landmark-tag poses
// (tests/data/aruco_loc): height above the table and downward tilt. The field of
// view comes from the calibration instead, so any lens works.
#define CAMERA_HEIGHT_MM 218.0
#define CAMERA_PITCH_DEG 44.5

namespace vision {

// Camera field pose: x/y in millimetres, z the height above the ground, heading
// in degrees normalized to ]-180, 180].
struct CameraPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0;
};

// Base for the two localisers. `Cam` holds each concrete localiser and there is
// no virtual dispatch; this base only carries the odometry prior, which the
// feature matcher uses to restrict its search (the marker localiser ignores it).
class CamLocalizer {
public:
    void setPrior(const CameraPosition& prior) { prior_ = prior; }

protected:
    const CameraPosition& prior() const { return prior_; }

private:
    CameraPosition prior_;
};

} // namespace vision
