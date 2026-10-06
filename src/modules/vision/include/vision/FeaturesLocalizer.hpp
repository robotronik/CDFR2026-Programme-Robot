#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "vision/CamLocalizer.hpp"

namespace vision {

// The constant pixel -> ground mapping of a capture, built from the camera
// calibration and the mounting. It is the same for every frame of a given size,
// so it is built once and reused.
//
// Ground positions are reported in the robot frame: x forwards, y left, origin
// on the ground directly under the camera.
struct GroundWarper {
    double mm_per_px = 4.0;
    double x0 = 0.0; // ground bounds, millimetres
    double y0 = 0.0;
    cv::Size out_size;
    cv::Mat homography; // CV_64F 3x3, capture pixel -> ground mm

    // The capture as a top-down patch, at this warper's mm per pixel.
    cv::Mat warp(const cv::Mat& gray) const;

    // Rectified patch pixels -> robot-frame (x forwards, y left) millimetres.
    cv::Point2d robotMm(const cv::Point2d& px) const;

    // The patch pixel directly under the camera.
    cv::Point2d cameraPatchPx() const;
};

// Localises the robot by matching ground features between a rectified capture
// and the field map, following PythonVision/features/features.py.
//
// The map's keypoints are computed once. Each capture is rectified to the same
// millimetre-per-pixel grid as the map, so a match between the two is a rigid
// motion with no scale to estimate; RANSAC fits that motion and the camera's
// field pose falls straight out of it.
class FeaturesLocalizer : public CamLocalizer {
public:
    FeaturesLocalizer();

    // Camera intrinsics; the capture -> ground warp is built from camera_matrix.
    bool loadCalibration(const std::string& cameraFilePath);
    // The reference map to match against (a 1 mm/px grayscale field image).
    bool loadMap(const std::string& mapFilePath);

    // Rectifies `frame`, matches it against the map, and fills `position` on a
    // successful fit. Returns false when no fix could be trusted.
    bool locate(const cv::Mat& frame, CameraPosition& position);

private:
    bool buildWarper(const cv::Size& size);

    cv::Mat camera_matrix_;
    bool calibrated_ = false;

    GroundWarper warper_;
    cv::Size warper_size_;

    cv::Mat map_gray_;
    double map_mm_per_px_ = 1.0;
    cv::Point2d map_origin_px_;
    std::vector<cv::KeyPoint> map_keypoints_;
    cv::Mat map_descriptors_;
    std::vector<cv::Point2d> map_points_px_;

    cv::Ptr<cv::AKAZE> detector_;
    cv::Ptr<cv::BFMatcher> matcher_;
};

} // namespace vision
