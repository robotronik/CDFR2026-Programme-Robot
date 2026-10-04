#pragma once

// Which localiser the camera runs. Both answer the same question - where is the
// robot on the field - so the rest of the program is unaffected by the choice.
//   1 = ArUco landmark tags (ids 20..23 at the table corners)
//   0 = ground-feature matching against the field map
#ifndef USE_ARUCO_LOCALISATION
#define USE_ARUCO_LOCALISATION 1
#endif

// --- Field ----------------------------------------------------------------

#define FIELD_WIDTH_MM 2000.0
#define FIELD_HEIGHT_MM 3000.0

// --- Camera mounting ------------------------------------------------------

// How high the camera sits above the table and how far it tilts down. The
// height and pitch are what lift a capture onto the ground plane; the field of
// view comes from the camera calibration instead, so it works for any lens.
#define CAMERA_HEIGHT_MM 233.2
#define CAMERA_PITCH_DEG 45.0

namespace vision {

// Where the camera is, in field millimetres, and which way it looks.
//
// `z` is the camera's height above the ground. `heading` is in degrees,
// normalized to ]-180, 180].
struct CameraPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0;
};

// Shared state for the two localisers.
//
// Deliberately not polymorphic: USE_ARUCO_LOCALISATION picks the implementation
// at compile time, so `Cam` holds the concrete localiser and there is no
// virtual dispatch. This base only carries the odometry prior, which the
// feature matcher uses to restrict its search and which the marker localiser
// has no use for.
class CamLocalizer {
public:
    void setPrior(const CameraPosition& prior) {
        prior_ = prior;
        hasPrior_ = true;
    }
    void clearPrior() { hasPrior_ = false; }

protected:
    bool hasPrior() const { return hasPrior_; }
    const CameraPosition& prior() const { return prior_; }

private:
    CameraPosition prior_;
    bool hasPrior_ = false;
};

} // namespace vision
