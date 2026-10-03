#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "vision/ArucoLocalizer.hpp"

// A game element is a marker with ArUco id 13. Its position is on the table, in
// millimetres, with its heading in degrees.
struct GameElement {
    int id = 13;
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
};

// Captures frames on its own thread and detects ArUco markers. It answers two
// questions: where the camera is on the table, and where the game elements are.
class ArucoCam {
public:
    ArucoCam(int camNumber, const char* calibrationFilePath);
    ~ArucoCam();

    ArucoCam(const ArucoCam&) = delete;
    ArucoCam& operator=(const ArucoCam&) = delete;

    // Starts/stops the capture thread. A negative camNumber emulates a camera
    // that is never started.
    void start();
    void stop();
    bool isEmulated() const { return id_ < 0; }

    // Latest camera localisation on the table. Returns true when one is known.
    bool getLocalisation(double& x, double& y, double& a) const;

    // Game elements seen in the latest frame, placed on the table using the
    // given robot/camera pose.
    std::vector<GameElement> getGameElements(double x, double y, double a) const;

private:
    void workerLoop();

    int id_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;
    vision::ArucoLocalizer localizer_;

    mutable std::mutex mutex_;
    std::vector<vision::DetectionResult> detections_;
    bool hasLocalisation_ = false;
    double localisationX_ = 0.0;
    double localisationY_ = 0.0;
    double localisationA_ = 0.0;
};
