#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vision/ArucoDetector.hpp"
#include "vision/CamLocalizer.hpp"

namespace vision {

/**
 * Localises the robot from the four fixed landmark tags (ids 20..23) at the
 * table corners, which are 100 mm squares.
 *
 * A single visible tag is enough to fix the camera. When several are visible at
 * once their estimates are averaged, the heading circularly.
 */
class ArucoLocalizer : public CamLocalizer {
public:
    explicit ArucoLocalizer(double tagSideMm = 100.0);

    bool loadCalibration(const std::string& calibrationFilePath);

    bool initCamera(int deviceIndex = 0, int width = 1280, int height = 800);
    void releaseCamera();
    bool isCameraOpen() const { return detector_.isCameraOpen(); }

    // Detects the landmark tags on `frame`. Returns true and fills `position`
    // when at least one of them yields a usable pose, false otherwise.
    bool locate(const cv::Mat& frame, CameraPosition& position);

    // Position from already-computed detections; lets a caller that also needs
    // the markers (role tagging, game elements) detect only once.
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
