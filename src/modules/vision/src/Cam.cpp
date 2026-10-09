#include "vision/Cam.hpp"

#include <chrono>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/logger.hpp"

namespace {

constexpr int kCameraWidth = 1280;
constexpr int kCameraHeight = 800;

// Game element tag (id 13): the tag side sets the pose scale, the cube side
// places the tag above the cube centre.
constexpr int kGameElementId = 13;
constexpr double kGameElementTagMm = GAME_ELEMENT_TAG_MM;
constexpr double kGameElementSideMm = GAME_ELEMENT_SIDE_MM;

// The field landmark tags (ids 20..23) are 100 mm squares.
constexpr double kLandmarkTagMm = 100.0;

constexpr double kDegToRad = M_PI / 180.0;

double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

// Rotation from the camera's optical frame (x right, y down, z forward) to the
// table frame (x/y in the plane, z up), looking along `headingDeg` and pitched
// `pitchDeg` down.
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

// Draws the detections on `image`, colour-coded by role.
void drawDetections(cv::Mat& image, const std::vector<vision::DetectionResult>& detections) {
    if (image.channels() == 1) {
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    }

    for (const vision::DetectionResult& detection : detections) {
        if (detection.corners.size() < 4) {
            continue;
        }

        cv::Scalar color(160, 160, 160);
        if (detection.id == kGameElementId) {
            color = cv::Scalar(0, 165, 255); // game element, orange
        } else if (vision::ArucoLocalizer::fieldPosition(detection.id) != nullptr) {
            color = cv::Scalar(0, 255, 0); // landmark tag, green
        }

        const std::vector<cv::Point> outline(detection.corners.begin(), detection.corners.end());
        cv::polylines(image, std::vector<std::vector<cv::Point>>{outline}, true, color, 2, cv::LINE_AA);

        // Label above the first corner (top-left by convention).
        const cv::Point labelAt = outline[0] + cv::Point(0, -10);
        cv::putText(image, std::to_string(detection.id), labelAt,
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv::LINE_AA);
    }
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
    LOG_DEBUG("Camera pose: (%.2f, %.2f, %.2f) -> Robot pose: (%.2f, %.2f, %.2f)", cameraPose.x, cameraPose.y, cameraPose.a, robotPose.x, robotPose.y, robotPose.a);
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

Cam::Cam(int camNumber, const char* calibrationFilePath, const char* mapFilePath) {
    id_ = camNumber;
    if (id_ < 0) {
        LOG_INFO("Emulating Cam");
        return;
    }

    if (!detector_.loadCalibration(calibrationFilePath)) {
        LOG_ERROR("Cam ", id_, " failed to load calibration from ", calibrationFilePath);
    }
    detector_.setMarkerSize(kGameElementId, kGameElementTagMm);
    for (int tagId : {20, 21, 22, 23}) {
        detector_.setMarkerSize(tagId, kLandmarkTagMm);
    }

    if (!arucoLocalizer_.loadCalibration(calibrationFilePath)) {
        LOG_ERROR("Cam ", id_, " failed to load the landmark localiser calibration");
    }
    if (!featuresLocalizer_.loadCalibration(calibrationFilePath)) {
        LOG_ERROR("Cam ", id_, " failed to load the feature localiser calibration");
    }
    if (!featuresLocalizer_.loadMap(mapFilePath)) {
        LOG_ERROR("Cam ", id_, " failed to load the feature map from ", mapFilePath);
    }
}

Cam::~Cam() {
    stop();
}

void Cam::start() {
    if (id_ < 0 || running_.load()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        detections_.clear();
        hasLocalisation_ = false;
        frame_.release();
    }

    running_.store(true);
    worker_ = std::thread(&Cam::workerLoop, this);
    LOG_GREEN_INFO("Cam ", id_, " started");
}

void Cam::stop() {
    running_.store(false);
    // The worker thread releases the camera itself (see workerLoop): libcamera
    // requires the device to be closed on the same thread that opened it.
    if (worker_.joinable()) {
        worker_.join();
    }
}

void Cam::setPrior(const position_t& robotPose) {
    std::lock_guard<std::mutex> lock(mutex_);
    priorRobot_ = robotPose;
    hasPrior_ = true;
}

void Cam::workerLoop() {
    // Open the camera on the capture thread: libcamera requires every camera
    // operation (open, configure, start, dispatch) to run on one thread.
    if (!detector_.isCameraOpen() && !detector_.initCamera(id_, kCameraWidth, kCameraHeight)) {
        LOG_ERROR("Cam ", id_, " failed to open camera");
        running_.store(false);
        return;
    }

    while (running_.load()) {
        cv::Mat frame;
        if (!detector_.captureFrame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        const std::vector<vision::DetectionResult> detections = detector_.detect(frame);

        // Snapshot the prior so setPrior() from another thread cannot race the
        // feature localiser.
        position_t priorRobot;
        bool hasPrior;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            priorRobot = priorRobot_;
            hasPrior = hasPrior_;
        }

        vision::CameraPosition arucoPosition;
        const bool hasAruco = arucoLocalizer_.locate(detections, arucoPosition);

        if (hasPrior) {
            const position_t cameraPrior = robotToCamera(priorRobot);
            featuresLocalizer_.setPrior({cameraPrior.x, cameraPrior.y, CAMERA_HEIGHT_MM,
                                         cameraPrior.a});
        }
        vision::CameraPosition featurePosition;
        const bool hasFeature = featuresLocalizer_.locate(frame, featurePosition, hasPrior);

        // Prefer the feature result, else the markers.
        const bool localised = hasFeature || hasAruco;
        const vision::CameraPosition& position = hasFeature ? featurePosition : arucoPosition;

        std::lock_guard<std::mutex> lock(mutex_);
        detections_ = detections;
        frame_ = frame;
        hasLocalisation_ = localised;
        if (localised) {
            localisation_.x = position.x;
            localisation_.y = position.y;
            localisation_.a = position.heading;
        }
    }

    // Released on the opening thread (see the note above).
    detector_.releaseCamera();
}

bool Cam::getLocalisation(position_t& cameraPose) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasLocalisation_) {
        return false;
    }
    cameraPose = localisation_;
    return true;
}

bool Cam::getPreview(std::vector<uchar>& jpeg) const {
    cv::Mat annotated;
    std::vector<vision::DetectionResult> detections;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frame_.empty()) {
            LOG_DEBUG("Cam ", id_, " has no frame to preview");
            return false;
        }
        // Encode outside the lock so the capture thread is not stalled.
        annotated = frame_.clone();
        detections = detections_;
    }

    drawDetections(annotated, detections);
    return cv::imencode(".jpg", annotated, jpeg);
}

bool Cam::getRawPreview(std::vector<uchar>& jpeg) const {
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frame_.empty()) {
            return false;
        }
        // Shallow copy: the capture thread rebinds frame_ instead of writing into
        // the buffer this reference points to.
        frame = frame_;
    }

    return cv::imencode(".jpg", frame, jpeg);
}

bool Cam::gameElementFromTag(const vision::DetectionResult& detection,
                             const position_t& cameraPose,
                             GameElement& element) {
    if (detection.id != kGameElementId || !detection.hasPose) {
        return false;
    }

    const cv::Matx33d cameraToTable = cameraToTableRotation(cameraPose.a, CAMERA_PITCH_DEG);
    const cv::Vec3d cameraPosition{cameraPose.x, cameraPose.y, CAMERA_HEIGHT_MM};

    cv::Matx33d elementToCamera;
    cv::Rodrigues(detection.rvec, elementToCamera);
    const cv::Matx33d elementToTable = cameraToTable * elementToCamera;

    // Lift the marker pose from the optical frame into the table frame.
    const cv::Vec3d tagPosition = cameraPosition + cameraToTable * detection.tvec;

    // The cube centre is half a side inwards along the tag's outward normal.
    const cv::Vec3d normal = elementToTable * cv::Vec3d(0.0, 0.0, 1.0);
    const cv::Vec3d cubePosition = tagPosition - (kGameElementSideMm / 2.0) * normal;

    cv::Mat rqR, rqQ;
    const cv::Vec3d euler = cv::RQDecomp3x3(elementToTable, rqR, rqQ);

    element.x = cubePosition[0];
    element.y = cubePosition[1];
    element.z = cubePosition[2];
    element.roll = euler[0];
    element.pitch = euler[1];
    element.yaw = euler[2];
    return true;
}

std::vector<GameElement> Cam::getGameElements(const position_t& cameraPose) const {
    std::vector<GameElement> elements;

    std::lock_guard<std::mutex> lock(mutex_);
    for (const vision::DetectionResult& detection : detections_) {
        GameElement element;
        if (gameElementFromTag(detection, cameraPose, element)) {
            elements.push_back(element);
        }
    }

    return elements;
}
