#include "vision/ArucoCam.hpp"

#include <chrono>
#include <cmath>

#include <opencv2/calib3d.hpp>

#include "utils/logger.hpp"

namespace {

constexpr int kCameraWidth = 1280;
constexpr int kCameraHeight = 800;

// The game elements carry ArUco id 13. `kGameElementSideMm` is the physical
// side of that marker, which sets the scale of its estimated pose.
constexpr int kGameElementId = 13;
constexpr double kGameElementSideMm = 80.0;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

} // namespace

void cameraToRobot(double& x, double& y, double& a) {
    const double robotA = a - OFFSET_CAM_A;
    const double rad = robotA * kDegToRad;
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    x -= OFFSET_CAM_X * c - OFFSET_CAM_Y * s;
    y -= OFFSET_CAM_X * s + OFFSET_CAM_Y * c;
    a = normalizeAngle(robotA);
}

void robotToCamera(double& x, double& y, double& a) {
    const double rad = a * kDegToRad;
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    x += OFFSET_CAM_X * c - OFFSET_CAM_Y * s;
    y += OFFSET_CAM_X * s + OFFSET_CAM_Y * c;
    a = normalizeAngle(a + OFFSET_CAM_A);
}

ArucoCam::ArucoCam(int camNumber, const char* calibrationFilePath) {
    id_ = camNumber;
    if (id_ < 0) {
        LOG_INFO("Emulating ArucoCam");
        return;
    }

    if (!localizer_.loadCalibration(calibrationFilePath)) {
        LOG_ERROR("ArucoCam ", id_, " failed to load calibration from ", calibrationFilePath);
    }
    localizer_.detector().setMarkerSize(kGameElementId, kGameElementSideMm);
}

ArucoCam::~ArucoCam() {
    stop();
}

void ArucoCam::start() {
    if (id_ < 0 || running_.load()) {
        return;
    }
    if (!localizer_.isCameraOpen() && !localizer_.initCamera(id_, kCameraWidth, kCameraHeight)) {
        LOG_ERROR("ArucoCam ", id_, " failed to open camera");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        detections_.clear();
        hasLocalisation_ = false;
    }

    running_.store(true);
    worker_ = std::thread(&ArucoCam::workerLoop, this);
    LOG_GREEN_INFO("ArucoCam ", id_, " started");
}

void ArucoCam::stop() {
    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
    if (id_ >= 0) {
        localizer_.releaseCamera();
    }
}

void ArucoCam::workerLoop() {
    while (running_.load()) {
        cv::Mat frame;
        if (!localizer_.detector().captureFrame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        std::vector<vision::DetectionResult> detections = localizer_.detector().detect(frame);

        std::lock_guard<std::mutex> lock(mutex_);
        detections_ = std::move(detections);

        // Localisation comes from the first landmark tag in the frame.
        for (const vision::DetectionResult& detection : detections_) {
            const cv::Point2d* field = vision::ArucoLocalizer::fieldPosition(detection.id);
            if (field == nullptr) {
                continue;
            }
            vision::CameraPosition position;
            if (!vision::ArucoLocalizer::cameraPositionForTag(detection, field->x, field->y, position)) {
                continue;
            }
            localisationX_ = position.x;
            localisationY_ = position.y;
            localisationA_ = position.heading;
            hasLocalisation_ = true;
            break;
        }
    }
}

bool ArucoCam::getLocalisation(double& x, double& y, double& a) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasLocalisation_) {
        return false;
    }
    x = localisationX_;
    y = localisationY_;
    a = localisationA_;
    return true;
}

std::vector<GameElement> ArucoCam::getGameElements(double x, double y, double a) const {
    std::vector<GameElement> elements;

    std::lock_guard<std::mutex> lock(mutex_);
    for (const vision::DetectionResult& detection : detections_) {
        if (detection.id != kGameElementId || !detection.hasPose) {
            continue;
        }

        cv::Matx33d rotation;
        cv::Rodrigues(detection.rvec, rotation);

        // Camera position in the marker frame, and the marker's heading, are
        // turned into the element's field position from the given pose.
        const cv::Vec3d camera = -(rotation.t() * detection.tvec);
        const double yaw = std::atan2(-rotation(0, 1), rotation(0, 0)) * kRadToDeg;

        GameElement element;
        element.id = detection.id;
        element.x = x + camera[1];
        element.y = y - camera[0];
        element.a = normalizeAngle(a + yaw - 180.0);
        elements.push_back(element);
    }

    return elements;
}
