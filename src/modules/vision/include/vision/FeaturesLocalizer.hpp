#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "vision/CamLocalizer.hpp"

namespace vision {

// Constant pixel -> ground mapping of a capture, built from the calibration and
// the mounting and reused for every frame of the same size. Ground positions are
// in the robot frame: x forwards, y left, origin under the camera.
struct GroundWarper {
    double mm_per_px = 4.0;
    double x0 = 0.0; // ground bounds, millimetres
    double y0 = 0.0;
    cv::Size out_size;
    cv::Mat homography; // CV_64F 3x3, capture pixel -> ground mm

    cv::Mat warp(const cv::Mat& gray) const;          // capture -> top-down patch
    cv::Point2d robotMm(const cv::Point2d& px) const; // patch pixel -> robot mm
    cv::Point2d cameraPatchPx() const;                // patch pixel under the camera
};

// Localises the robot by matching ground features between a rectified capture and
// the field map. The map keypoints are computed once; each capture is rectified
// to the same mm/px grid, so a match is a rigid motion (no scale) that RANSAC
// fits, from which the camera's field pose follows.
class FeaturesLocalizer : public CamLocalizer {
public:
    FeaturesLocalizer();

    // Camera intrinsics; the capture -> ground warp is built from camera_matrix.
    bool loadCalibration(const std::string& cameraFilePath);
    // The 1 mm/px grayscale field image to match against.
    bool loadMap(const std::string& mapFilePath);

    // Rectifies `frame`, matches it against the map, and fills `position` on a
    // successful fit. When `hasStartingPosition` is true the match is restricted
    // to the window around the stored prior, otherwise the whole map is offered.
    bool locate(const cv::Mat& frame, CameraPosition& position, bool hasStartingPosition);

private:
    bool buildWarper(const cv::Size& size);

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
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
