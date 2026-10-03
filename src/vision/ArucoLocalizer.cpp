#include "vision/ArucoLocalizer.hpp"

#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "utils/logger.hpp"

namespace vision {

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

// The field's landmark tags, ids 20..23, at the table corners.
const std::map<int, cv::Point2d>& tagFieldPositions() {
    static const std::map<int, cv::Point2d> positions = {
        {20, {-400.0, -900.0}}, // Top-right
        {21, {-400.0,  900.0}}, // Top-left
        {22, { 400.0, -900.0}}, // Bottom-left
        {23, { 400.0,  900.0}}, // Bottom-right
    };
    return positions;
}

} // namespace

ArucoLocalizer::ArucoLocalizer(double tagSideMm) {
    for (const auto& entry : tagFieldPositions()) {
        detector_.setMarkerSize(entry.first, tagSideMm);
    }
}

bool ArucoLocalizer::loadCalibration(const std::string& calibrationFilePath) {
    return detector_.loadCalibration(calibrationFilePath);
}

bool ArucoLocalizer::initCamera(int deviceIndex, int width, int height) {
    return detector_.initCamera(deviceIndex, width, height);
}

void ArucoLocalizer::releaseCamera() {
    detector_.releaseCamera();
}

const cv::Point2d* ArucoLocalizer::fieldPosition(int tagId) {
    const std::map<int, cv::Point2d>& positions = tagFieldPositions();
    const auto it = positions.find(tagId);
    return it == positions.end() ? nullptr : &it->second;
}

bool ArucoLocalizer::cameraPositionForTag(const DetectionResult& detection,
                                          double tagFieldX, double tagFieldY,
                                          CameraPosition& position) {
    if (!detection.hasPose) {
        return false;
    }

    cv::Matx33d rotation;
    cv::Rodrigues(detection.rvec, rotation);

    // Camera position expressed in the marker frame: -R^T * tvec.
    const cv::Vec3d camera = -(rotation.t() * detection.tvec);

    // The tag is a known landmark, so the camera's field position follows from
    // it. The axis swap and signs are the field landmark convention, checked
    // against captures whose true pose is known.
    position.x = tagFieldX - camera[1];
    position.y = tagFieldY + camera[0];
    position.z = camera[2];

    // Heading is the rotation about the vertical axis, negated; the +180 turns
    // the marker's forward axis into the camera's.
    const double yaw = std::atan2(-rotation(0, 1), rotation(0, 0)) * kRadToDeg;
    position.heading = normalizeAngle(-yaw + 180.0);
    return true;
}

bool ArucoLocalizer::locate(const cv::Mat& frame, CameraPosition& position) {
    const std::vector<DetectionResult> detections = detector_.detect(frame);

    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    double sumCos = 0.0, sumSin = 0.0;
    int count = 0;

    for (const DetectionResult& detection : detections) {
        const cv::Point2d* field = fieldPosition(detection.id);
        if (field == nullptr) {
            continue;
        }

        CameraPosition estimate;
        if (!cameraPositionForTag(detection, field->x, field->y, estimate)) {
            continue;
        }

        sumX += estimate.x;
        sumY += estimate.y;
        sumZ += estimate.z;
        sumCos += std::cos(estimate.heading * kDegToRad);
        sumSin += std::sin(estimate.heading * kDegToRad);
        ++count;
    }

    if (count == 0) {
        return false;
    }

    position.x = sumX / count;
    position.y = sumY / count;
    position.z = sumZ / count;
    position.heading = std::atan2(sumSin, sumCos) * kRadToDeg;
    return true;
}

bool ArucoLocalizer::locate(CameraPosition& position) {
    cv::Mat frame;
    if (!detector_.captureFrame(frame)) {
        return false;
    }
    return locate(frame, position);
}

} // namespace vision
