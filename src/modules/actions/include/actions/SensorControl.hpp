#pragma once

#include "lidar/Lidar.hpp"
#include "mat/mat.hpp"
#include "vision/Cam.hpp"

class Arduino;

// The robot's sensors. Owns the lidar, the camera and the mat; the Arduino (and
// through it the low-level button/latch/limit inputs) is owned by the Robot and
// only referenced here.
class SensorControl {
    public:
        explicit SensorControl(Arduino* arduino);

        bool readButtonSensor();
        bool readLatchSensor();
        bool readLimitSwitchBottom();
        bool readLimitSwitchTop();

        Arduino* arduino() const { return arduino_; }
        Lidar& lidar() { return lidar_; }
        Cam& cam() { return cam_; }
        Mat& mat() { return mat_; }

    private:
        Arduino* arduino_;
        Lidar lidar_;
        Cam cam_;
        Mat mat_;
};
