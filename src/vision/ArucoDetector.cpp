#include "vision/ArucoDetector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/logger.hpp"

namespace vision {

namespace {

// ArUco dictionary used by the competition markers.
constexpr int kPredefinedDictionary = cv::aruco::DICT_4X4_50;

// Detection parameters, mirroring the previous Python service
// (pi_detect_aruco.py). The corner refinement window is left at its default
// (0.01) which matches the value previously set explicitly in Python.
void configureParameters(cv::aruco::DetectorParameters& p) {
    p.adaptiveThreshWinSizeMin = 3;
    p.adaptiveThreshWinSizeMax = 23;
    p.adaptiveThreshWinSizeStep = 10;
    p.adaptiveThreshConstant = 7;

    p.minMarkerPerimeterRate = 0.03;
    p.maxMarkerPerimeterRate = 4.0;
    p.polygonalApproxAccuracyRate = 0.03;
    p.minCornerDistanceRate = 0.05;
    p.minDistanceToBorder = 3;
    p.minMarkerDistanceRate = 0.05;

    p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
    p.cornerRefinementWinSize = 5;
    p.cornerRefinementMaxIterations = 30;
    p.cornerRefinementMinAccuracy = 0.1;

    p.markerBorderBits = 1;
    p.perspectiveRemovePixelPerCell = 4;
    p.perspectiveRemoveIgnoredMarginPerCell = 0.13;
    p.maxErroneousBitsInBorderRate = 0.35;
    p.minOtsuStdDev = 5.0;
    p.errorCorrectionRate = 0.6;

    // The game element tags are rendered inverted (white on black) in the
    // simulator, so the detector must accept inverted markers.
    p.detectInvertedMarker = true;
    p.useAruco3Detection = true;
}

std::vector<cv::Point3f> markerObjectPoints(double markerLength) {
    const float h = static_cast<float>(markerLength / 2.0);
    return {
        {-h,  h, 0.0f},
        { h,  h, 0.0f},
        { h, -h, 0.0f},
        {-h, -h, 0.0f},
    };
}

} // namespace

ArucoDetector::ArucoDetector() {
    auto params = cv::makePtr<cv::aruco::DetectorParameters>();
    configureParameters(*params);

#if ARUCO_OPENCV_NEW_API
    detector_ = cv::makePtr<cv::aruco::ArucoDetector>(
        cv::aruco::getPredefinedDictionary(kPredefinedDictionary), *params);
#else
    dictionary_ = cv::aruco::getPredefinedDictionary(kPredefinedDictionary);
    parameters_ = params;
#endif
}

ArucoDetector::~ArucoDetector() {
    releaseCamera();
}

bool ArucoDetector::loadCalibration(const std::string& calibrationFilePath) {
    cv::FileStorage fs(calibrationFilePath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOG_ERROR("ArucoDetector - failed to open calibration file ", calibrationFilePath);
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["dist_coeffs"] >> distCoeffs_;
    fs.release();

    if (cameraMatrix_.empty() || distCoeffs_.empty()) {
        LOG_ERROR("ArucoDetector - calibration file ", calibrationFilePath,
                  " is missing camera_matrix or dist_coeffs");
        cameraMatrix_.release();
        distCoeffs_.release();
        calibrated_ = false;
        return false;
    }

    calibrated_ = true;
    LOG_INFO("ArucoDetector - loaded calibration from ", calibrationFilePath);
    return true;
}

void ArucoDetector::setMarkerSize(int id, double size) {
    markerSizes_[id] = size;
}

void ArucoDetector::setMarkerSizes(const std::map<int, double>& sizes) {
    for (const auto& [id, size] : sizes) {
        markerSizes_[id] = size;
    }
}

void ArucoDetector::clearMarkerSizes() {
    markerSizes_.clear();
}

bool ArucoDetector::initCamera(int deviceIndex, int width, int height) {
    std::lock_guard<std::mutex> lock(captureMutex_);
    if (capture_.isOpened()) {
        capture_.release();
    }

    capture_.open(deviceIndex);
    if (!capture_.isOpened()) {
        LOG_ERROR("ArucoDetector - failed to open camera device ", deviceIndex);
        return false;
    }

    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    LOG_GREEN_INFO("ArucoDetector - camera ", deviceIndex, " opened at ",
                   static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)), "x",
                   static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT)));
    return true;
}

bool ArucoDetector::isCameraOpen() const {
    std::lock_guard<std::mutex> lock(captureMutex_);
    return capture_.isOpened();
}

void ArucoDetector::releaseCamera() {
    std::lock_guard<std::mutex> lock(captureMutex_);
    if (capture_.isOpened()) {
        capture_.release();
    }
}

bool ArucoDetector::captureFrame(cv::Mat& outFrame) {
    std::lock_guard<std::mutex> lock(captureMutex_);
    if (!capture_.isOpened()) {
        return false;
    }

    capture_ >> outFrame;
    if (outFrame.empty()) {
        outFrame.release();
        return false;
    }
    return true;
}

std::vector<DetectionResult> ArucoDetector::detect(const cv::Mat& frame) {
    std::vector<DetectionResult> results;
    if (frame.empty()) {
        return results;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = frame;
    }

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;

#if ARUCO_OPENCV_NEW_API
    detector_->detectMarkers(gray, markerCorners, ids, rejected);
#else
    cv::aruco::detectMarkers(gray, dictionary_, markerCorners, ids, parameters_, rejected);
#endif

    results.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        DetectionResult result;
        result.id = ids[i];
        result.corners = markerCorners[i];

        auto sizeIt = markerSizes_.find(result.id);
        if (sizeIt != markerSizes_.end() && calibrated_) {
            const std::vector<cv::Point3f> objectPoints = markerObjectPoints(sizeIt->second);
            cv::solvePnP(objectPoints, result.corners, cameraMatrix_, distCoeffs_,
                         result.rvec, result.tvec, false);
            result.hasPose = true;
        }

        results.push_back(std::move(result));
    }

    return results;
}

std::vector<DetectionResult> ArucoDetector::captureAndDetect() {
    cv::Mat frame;
    if (!captureFrame(frame)) {
        return {};
    }
    return detect(frame);
}

} // namespace vision
