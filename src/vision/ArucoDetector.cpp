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
    // A game element tag is drawn with a white margin around its marker. At the
    // default constant the detector locks onto that outer margin instead of the
    // marker itself, which flips its polarity; a higher constant makes it lock
    // onto the marker's own border.
    p.adaptiveThreshConstant = 20;

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

    p.detectInvertedMarker = false;
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

bool ArucoDetector::initCamera(int deviceIndex, int width, int height) {
    std::lock_guard<std::mutex> lock(captureMutex_);
    if (capture_.isOpened()) {
        capture_.release();
    }

    // On Linux the default backend is GStreamer, whose pipeline fails to start
    // for plain UVC webcams (including the OV9281 and laptop cameras), so
    // capture never produces a frame. V4L2 talks to the device directly; fall
    // back to the default backend when it is unavailable.
#ifdef __linux__
    capture_.open(deviceIndex, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
        capture_.open(deviceIndex);
    }
#else
    capture_.open(deviceIndex);
#endif
    if (!capture_.isOpened()) {
        LOG_ERROR("ArucoDetector - failed to open camera device ", deviceIndex);
        return false;
    }
    captureFailCount_ = 0;

    // A device only supports a fixed set of modes: the driver picks the closest
    // one, so the resolution read back is authoritative, not the request.
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    const int actualWidth = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int actualHeight = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    if (actualWidth != width || actualHeight != height) {
        LOG_WARNING("ArucoDetector - camera ", deviceIndex, " does not support ",
                    width, "x", height, ", using ", actualWidth, "x", actualHeight);
    }

    // The negotiated pixel format tells whether the backend can decode what the
    // device emits (a raw Bayer/Y10 node cannot be read as BGR).
    const int fourcc = static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));
    char fourccText[5] = {0};
    for (int i = 0; i < 4; ++i) {
        fourccText[i] = static_cast<char>((fourcc >> (8 * i)) & 0xFF);
    }
    LOG_GREEN_INFO("ArucoDetector - camera ", deviceIndex, " opened at ",
                   actualWidth, "x", actualHeight, " (", fourccText, ")");
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
        // An open device that never delivers a frame is the usual cause of a
        // missing preview, so report it instead of failing silently.
        if (captureFailCount_ == 0 || captureFailCount_ % 400 == 0) {
            LOG_WARNING("ArucoDetector - camera returned no frame (",
                        captureFailCount_ + 1, " failures)");
        }
        ++captureFailCount_;
        return false;
    }

    if (captureFailCount_ > 0) {
        LOG_GREEN_INFO("ArucoDetector - camera delivered a frame after ",
                       captureFailCount_, " empty reads");
        captureFailCount_ = 0;
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

} // namespace vision
