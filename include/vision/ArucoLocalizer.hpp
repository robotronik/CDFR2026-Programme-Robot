#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "vision/ArucoDetector.hpp"

namespace vision {

/**
 * Where the camera is, in field millimetres, and which way it looks.
 *
 * `z` is the camera's height above the ground. `heading` is in degrees,
 * normalized to ]-180, 180].
 */
struct CameraPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0;
};

/**
 * Wraps an ArucoDetector and turns a detected landmark into the camera's
 * position on the field.
 *
 * The four field tags (ids 20..23) are fixed landmarks at the table corners and
 * are 100 mm squares, so a single tag is enough to fix the camera. When several
 * are visible at once their estimates are averaged, the heading circularly.
 */
class ArucoLocalizer {
public:
    explicit ArucoLocalizer(double tagSideMm = 100.0);

    bool loadCalibration(const std::string& calibrationFilePath);

    bool initCamera(int deviceIndex = 0, int width = 1280, int height = 800);
    void releaseCamera();
    bool isCameraOpen() const { return detector_.isCameraOpen(); }

    // Detects the landmark tags on `frame`. Returns true and fills `position`
    // when at least one of them yields a usable pose, false otherwise.
    bool locate(const cv::Mat& frame, CameraPosition& position);

    // Field position (mm) of a known tag, or nullptr when the id is unknown.
    static const cv::Point2d* fieldPosition(int tagId);

    // Camera position implied by one pose-bearing detection of a known tag.
    static bool cameraPositionForTag(const DetectionResult& detection,
                                     double tagFieldX, double tagFieldY,
                                     CameraPosition& position);

    ArucoDetector& detector() { return detector_; }

private:
    ArucoDetector detector_;
};

} // namespace vision
