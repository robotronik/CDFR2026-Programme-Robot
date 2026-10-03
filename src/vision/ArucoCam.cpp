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

double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

// Rotation taking a vector from the camera's optical frame (x right, y down,
// z forward) to the table frame (x, y in the plane, z up), for a camera looking
// along `headingDeg` and pitched `pitchDeg` down.
cv::Matx33d cameraToTableRotation(double headingDeg, double pitchDeg) {
    const double a = headingDeg * kDegToRad;
    const double p = pitchDeg * kDegToRad;
    const double ca = std::cos(a), sa = std::sin(a);
    const double cp = std::cos(p), sp = std::sin(p);

    // Columns are the camera axes expressed in the table frame.
    return cv::Matx33d(
         sa, -sp * ca, cp * ca,
        -ca, -sp * sa, cp * sa,
          0,      -cp,     -sp);
}

} // namespace

position_t cameraToRobot(const position_t& cameraPose) {
    const double robotA = cameraPose.a - OFFSET_CAM_A;
    const double rad = robotA * kDegToRad;
    const double c = std::cos(rad);
    const double s = std::sin(rad);

    position_t robotPose = {0.0, 0.0, 0.0};
    robotPose.x = cameraPose.x - (OFFSET_CAM_X * c - OFFSET_CAM_Y * s);
    robotPose.y = cameraPose.y - (OFFSET_CAM_X * s + OFFSET_CAM_Y * c);
    robotPose.a = normalizeAngle(robotA);
    return robotPose;
}

position_t robotToCamera(const position_t& robotPose) {
    const double rad = robotPose.a * kDegToRad;
    const double c = std::cos(rad);
    const double s = std::sin(rad);

    position_t cameraPose = {0.0, 0.0, 0.0};
    cameraPose.x = robotPose.x + (OFFSET_CAM_X * c - OFFSET_CAM_Y * s);
    cameraPose.y = robotPose.y + (OFFSET_CAM_X * s + OFFSET_CAM_Y * c);
    cameraPose.a = normalizeAngle(robotPose.a + OFFSET_CAM_A);
    return cameraPose;
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

        // Localisation comes from the first landmark tag in the frame. A frame
        // without a usable tag invalidates the previous fix.
        hasLocalisation_ = false;
        for (const vision::DetectionResult& detection : detections_) {
            const cv::Point2d* field = vision::ArucoLocalizer::fieldPosition(detection.id);
            if (field == nullptr) {
                continue;
            }
            vision::CameraPosition position;
            if (!vision::ArucoLocalizer::cameraPositionForTag(detection, field->x, field->y, position)) {
                continue;
            }
            localisation_.x = position.x;
            localisation_.y = position.y;
            localisation_.a = position.heading;
            hasLocalisation_ = true;
            break;
        }
    }
}

bool ArucoCam::getLocalisation(position_t& cameraPose) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasLocalisation_) {
        return false;
    }
    cameraPose = localisation_;
    return true;
}

std::vector<GameElement> ArucoCam::getGameElements(const position_t& robotPose) const {
    const position_t cameraPose = robotToCamera(robotPose);
    const cv::Matx33d cameraToTable = cameraToTableRotation(cameraPose.a, CAMERA_PITCH_DEG);
    const cv::Vec3d cameraPosition{cameraPose.x, cameraPose.y, CAMERA_HEIGHT_MM};

    std::vector<GameElement> elements;

    std::lock_guard<std::mutex> lock(mutex_);
    for (const vision::DetectionResult& detection : detections_) {
        if (detection.id != kGameElementId || !detection.hasPose) {
            continue;
        }

        cv::Matx33d elementToCamera;
        cv::Rodrigues(detection.rvec, elementToCamera);

        // The detected marker pose is in the camera's optical frame; lift it
        // into the table frame through the camera's mounting.
        const cv::Vec3d position = cameraPosition + cameraToTable * detection.tvec;
        cv::Mat rqR, rqQ;
        const cv::Vec3d euler = cv::RQDecomp3x3(cameraToTable * elementToCamera, rqR, rqQ);

        GameElement element;
        element.x = position[0];
        element.y = position[1];
        element.z = position[2];
        element.roll = euler[0];
        element.pitch = euler[1];
        element.yaw = euler[2];
        elements.push_back(element);
    }

    return elements;
}
