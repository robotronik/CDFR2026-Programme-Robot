#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vision/ArucoDetector.hpp"
#include "vision/CamLocalizer.hpp"

namespace vision {

// Localises the robot from the four fixed landmark tags (ids 20..23, 100 mm
// squares) at the table corners. One visible tag is enough; several are averaged
// (heading circularly).
class ArucoLocalizer : public CamLocalizer {
public:
    explicit ArucoLocalizer(double tagSideMm = 100.0);

    bool loadCalibration(const std::string& calibrationFilePath);

    // Detects the landmark tags on `frame`. Returns true and fills `position`
    // when at least one tag yields a usable pose.
    bool locate(const cv::Mat& frame, CameraPosition& position);

    // Same as above from already-computed detections, so a caller that also
    // needs the markers detects only once.
    bool locate(const std::vector<DetectionResult>& detections,
                CameraPosition& position) const;

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
