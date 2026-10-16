#pragma once

#include <map>
#include <vector>

#include <opencv2/core.hpp>

#include "vision/ArucoDetector.hpp"
#include "vision/CameraGeometry.hpp"

namespace vision {

// Localises the robot from the four fixed landmark tags (ids 20..23, 100 mm
// squares) at the table corners. One visible tag is enough; several are averaged
// (heading circularly). Cam detects the markers once and hands the detections
// over, so this holds no state of its own.
class ArucoLocalizer {
public:
    // Side length of a landmark tag, in millimetres.
    static constexpr double kLandmarkTagMm = 100.0;

    // The four landmark tags at the table corners, id -> field position in
    // millimetres. The one place the field tags are defined.
    static const std::map<int, cv::Point2d>& landmarkTags();

    // Registers the landmark tag sizes on `detector`, so a caller that detects
    // the markers itself uses the size the localiser assumes.
    static void registerLandmarkTags(ArucoDetector& detector);

    // Camera position from already-computed detections. False when none of them
    // is a pose-bearing landmark tag.
    static bool locate(const std::vector<DetectionResult>& detections,
                       CameraPosition& position);

    // Field position (mm) of a known tag, or nullptr when the id is unknown.
    static const cv::Point2d* fieldPosition(int tagId);

private:
    // Camera position implied by one pose-bearing detection of a known tag.
    static bool cameraPositionForTag(const DetectionResult& detection,
                                     double tagFieldX, double tagFieldY,
                                     CameraPosition& position);
};

} // namespace vision
