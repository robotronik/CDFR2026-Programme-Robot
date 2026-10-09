#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "drive_interface.h" // for position_t
#include "vision/ArucoDetector.hpp"
#include "vision/ArucoLocalizer.hpp"
#include "vision/CamLocalizer.hpp"
#include "vision/FeaturesLocalizer.hpp"

// Camera offset in the robot frame, in millimetres and degrees.
#define OFFSET_CAM_X 129
#define OFFSET_CAM_Y 4.5
#define OFFSET_CAM_A 0

// Game elements are 110 mm cubes whose faces carry an 80 mm ArUco id 13 marker;
// the marker size sets the pose scale, the cube side places the marker above the
// cube centre.
#define GAME_ELEMENT_SIDE_MM 110.0
#define GAME_ELEMENT_TAG_MM 80.0

// A game element's table pose in millimetres and degrees. The yaw follows the
// marker's own axes (a flat marker oriented like the field tags reads ~90 deg).
struct GameElement {
    // TODO: reuse position_t and model height as steps (1,2,3 + a vertical flag),
    // moving this to structs.hpp so the mat logic can share it.
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// Table-pose conversion between the camera frame and the robot frame, which are
// offset by (OFFSET_CAM_X, OFFSET_CAM_Y) and OFFSET_CAM_A.
position_t cameraToRobot(const position_t& cameraPose);
position_t robotToCamera(const position_t& robotPose);

// Captures frames on its own thread and localises the camera on the field. Both
// localisers run every frame; when both succeed the feature result is reported.
// ArUco detection also feeds the game elements and the preview.
class Cam {
public:
    Cam(int camNumber, const char* calibrationFilePath, const char* mapFilePath);
    ~Cam();

    Cam(const Cam&) = delete;
    Cam& operator=(const Cam&) = delete;

    // Starts/stops the capture thread. A negative camNumber emulates a camera
    // that is never started.
    void start();
    void stop();

    // Odometry estimate of the robot's pose; restricts the feature search.
    void setPrior(const position_t& robotPose);

    // Latest camera pose on the table (use cameraToRobot() for the robot pose).
    bool getLocalisation(position_t& cameraPose) const;

    // JPEG of the latest frame with the detected markers outlined and labelled.
    bool getPreview(std::vector<uchar>& jpeg) const;

    // JPEG of the latest frame without any overlay.
    bool getRawPreview(std::vector<uchar>& jpeg) const;

    // Controls
    void setExposureValue(float ev) { detector_.setExposureValue(ev); }
    void setContrast(float contrast) { detector_.setContrast(contrast); }
    void setBrightness(float brightness) { detector_.setBrightness(brightness); }
    float getExposureValue() const { return detector_.getExposureValue(); }
    float getContrast() const { return detector_.getContrast(); }
    float getBrightness() const { return detector_.getBrightness(); }

    // Game elements in the latest frame, placed from the given camera pose.
    std::vector<GameElement> getGameElements(const position_t& cameraPose) const;

    // Places one detection as the centre of its cube. False when it is not a
    // game element with a usable pose.
    static bool gameElementFromTag(const vision::DetectionResult& detection,
                                   const position_t& cameraPose,
                                   GameElement& element);

private:
    void workerLoop();

    int id_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;

    // Owns the capture device; detects the markers for the preview and elements.
    vision::ArucoDetector detector_;
    vision::ArucoLocalizer arucoLocalizer_;
    vision::FeaturesLocalizer featuresLocalizer_;

    mutable std::mutex mutex_;
    std::vector<vision::DetectionResult> detections_;
    cv::Mat frame_;
    bool hasLocalisation_ = false;
    position_t localisation_ = {0.0, 0.0, 0.0};

    bool hasPrior_ = false;
    position_t priorRobot_ = {0.0, 0.0, 0.0};
};
