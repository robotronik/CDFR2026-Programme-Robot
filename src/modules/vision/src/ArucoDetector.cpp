#include "vision/ArucoDetector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#ifdef CAMERA_USE_LIBCAMERA
#  include <libcamera/formats.h>

#  include "vision/RawFrame.hpp"
#endif

#include "utils/logger.hpp"

namespace vision {

namespace {

// ArUco dictionary used by the competition markers.
constexpr int kPredefinedDictionary = cv::aruco::DICT_4X4_50;

// Detection parameters, mirroring the previous Python service
// (pi_detect_aruco.py).
void configureParameters(cv::aruco::DetectorParameters& p) {
    p.adaptiveThreshWinSizeMin = 3;
    p.adaptiveThreshWinSizeMax = 23;
    p.adaptiveThreshWinSizeStep = 10;
    // Low-contrast tags need a small adaptive-threshold constant (13); larger
    // values missed the real captures in tests/data/aruco_loc.
    p.adaptiveThreshConstant = 13;

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

// Report a device that opened but never delivers frames occasionally, not once
// per failed read.
constexpr int kCaptureFailReportInterval = 400;

void reportCaptureFailure(int previousFailures) {
    if (previousFailures == 0 || previousFailures % kCaptureFailReportInterval == 0) {
        LOG_WARNING("camera returned no frame (", previousFailures + 1, " failures)");
    }
}

#ifdef CAMERA_USE_LIBCAMERA
// Converts one libcamera frame into a grayscale cv::Mat: the whole pipeline
// works on luminance only. The result owns its pixels, so it stays valid after
// the camera buffer is recycled. Returns false for formats it cannot unpack.
bool convertToGray(const RawFrame& raw, cv::Mat& out) {
    if (raw.data == nullptr || raw.width <= 0 || raw.height <= 0) {
        return false;
    }

    switch (raw.format) {
    case libcamera::formats::R8: {
        const cv::Mat gray(raw.height, raw.width, CV_8UC1, const_cast<uint8_t*>(raw.data), raw.stride);
        out = gray.clone();
        return true;
    }
    case libcamera::formats::R10:
    case libcamera::formats::R12:
    case libcamera::formats::R16: {
        const cv::Mat gray16(raw.height, raw.width, CV_16UC1, const_cast<uint8_t*>(raw.data), raw.stride);
        gray16.convertTo(out, CV_8U, 1.0 / 256.0);
        return true;
    }
    case libcamera::formats::BGRA8888:
    case libcamera::formats::RGBA8888:
#ifdef CAMERA_LIBCAMERA_HAVE_XRGB8888
    case libcamera::formats::ARGB8888:
    case libcamera::formats::ABGR8888:
    case libcamera::formats::XRGB8888:
    case libcamera::formats::XBGR8888:
#endif
    {
        const cv::Mat bgra(raw.height, raw.width, CV_8UC4, const_cast<uint8_t*>(raw.data), raw.stride);
        // libcamera's "RGBA" is stored R,G,B,A in memory, i.e. BGRA byte order;
        // the XRGB/ARGB groups use the opposite order.
        if (raw.format == libcamera::formats::RGBA8888 ||
            raw.format == libcamera::formats::BGRA8888) {
            cv::cvtColor(bgra, out, cv::COLOR_BGRA2GRAY);
        } else {
            cv::cvtColor(bgra, out, cv::COLOR_RGBA2GRAY);
        }
        return true;
    }
    case CAMERA_FORMAT_BGR24: {
        const cv::Mat bgr(raw.height, raw.width, CV_8UC3, const_cast<uint8_t*>(raw.data), raw.stride);
        cv::cvtColor(bgr, out, cv::COLOR_BGR2GRAY);
        return true;
    }
    case CAMERA_FORMAT_RGB24: {
        const cv::Mat rgb(raw.height, raw.width, CV_8UC3, const_cast<uint8_t*>(raw.data), raw.stride);
        cv::cvtColor(rgb, out, cv::COLOR_RGB2GRAY);
        return true;
    }
    case libcamera::formats::YUYV: {
        const cv::Mat yuyv(raw.height, raw.width, CV_8UC2, const_cast<uint8_t*>(raw.data), raw.stride);
        cv::cvtColor(yuyv, out, cv::COLOR_YUV2GRAY_YUYV);
        return true;
    }
    case libcamera::formats::UYVY: {
        const cv::Mat uyvy(raw.height, raw.width, CV_8UC2, const_cast<uint8_t*>(raw.data), raw.stride);
        cv::cvtColor(uyvy, out, cv::COLOR_YUV2GRAY_UYVY);
        return true;
    }
    case libcamera::formats::NV12:
    case libcamera::formats::NV21:
    case libcamera::formats::YUV420: {
        // The ISP's Y plane alone: the chroma planes of the monochrome OV9281
        // are pure noise, and the detector only needs luminance.
        const cv::Mat gray(raw.height, raw.width, CV_8UC1, const_cast<uint8_t*>(raw.data), raw.stride);
        out = gray.clone();
        return true;
    }
    default:
        return false;
    }
}
#endif

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
        LOG_ERROR("failed to open calibration file ", calibrationFilePath);
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["dist_coeffs"] >> distCoeffs_;
    if (!fs["exposure_value"].empty()) {
        fs["exposure_value"] >> exposureValue_;
    }
    if (!fs["contrast"].empty()) {
        fs["contrast"] >> contrast_;
    }
    if (!fs["brightness"].empty()) {
        fs["brightness"] >> brightness_;
    }
    fs.release();

    if (cameraMatrix_.empty() || distCoeffs_.empty()) {
        LOG_ERROR("calibration file ", calibrationFilePath,
                  " is missing camera_matrix or dist_coeffs");
        cameraMatrix_.release();
        distCoeffs_.release();
        calibrated_ = false;
        return false;
    }

    calibrated_ = true;
    LOG_INFO("loaded calibration from ", calibrationFilePath,
             " (EV: ", exposureValue_, ", contrast: ", contrast_, ", brightness: ", brightness_, ")");
    return true;
}

void ArucoDetector::setMarkerSize(int id, double size) {
    markerSizes_[id] = size;
}

void ArucoDetector::setExposureValue(float ev) {
    std::lock_guard<std::mutex> lock(captureMutex_);
    exposureValue_ = ev;
#ifdef CAMERA_USE_LIBCAMERA
    if (libcamera_) {
        libcamera_->setExposureValue(ev);
    }
#endif
}

void ArucoDetector::setContrast(float contrast) {
    std::lock_guard<std::mutex> lock(captureMutex_);
    contrast_ = contrast;
#ifdef CAMERA_USE_LIBCAMERA
    if (libcamera_) {
        libcamera_->setContrast(contrast);
    }
#endif
}

void ArucoDetector::setBrightness(float brightness) {
    std::lock_guard<std::mutex> lock(captureMutex_);
    brightness_ = brightness;
#ifdef CAMERA_USE_LIBCAMERA
    if (libcamera_) {
        libcamera_->setBrightness(brightness);
    }
#endif
}

bool ArucoDetector::initCamera(int deviceIndex, int width, int height) {
    std::lock_guard<std::mutex> lock(captureMutex_);

#ifdef CAMERA_USE_LIBCAMERA
    if (libcamera_ && libcamera_->isOpen()) {
        libcamera_->close();
    }
    libcamera_ = std::make_unique<LibcameraCamera>();
    libcamera_->setExposureValue(exposureValue_);
    libcamera_->setContrast(contrast_);
    libcamera_->setBrightness(brightness_);
    if (!libcamera_->open(deviceIndex, width, height)) {
        LOG_ERROR("failed to open libcamera device ", deviceIndex);
        libcamera_.reset();
        return false;
    }
    captureFailCount_ = 0;
    return true;
#else
    if (capture_.isOpened()) {
        capture_.release();
    }

    // On Linux the default backend is GStreamer, whose pipeline fails to start
    // for plain UVC webcams, so talk to the device through V4L2 and fall back to
    // the default backend when it is unavailable.
#ifdef __linux__
    capture_.open(deviceIndex, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
        capture_.open(deviceIndex);
    }
#else
    capture_.open(deviceIndex);
#endif
    if (!capture_.isOpened()) {
        LOG_ERROR("failed to open camera device ", deviceIndex);
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
        LOG_WARNING("camera ", deviceIndex, " does not support ",
                    width, "x", height, ", using ", actualWidth, "x", actualHeight);
    }

    // The negotiated pixel format tells whether the backend can decode what the
    // device emits (a raw Bayer/Y10 node cannot be read as BGR).
    const int fourcc = static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));
    char fourccText[5] = {0};
    for (int i = 0; i < 4; ++i) {
        fourccText[i] = static_cast<char>((fourcc >> (8 * i)) & 0xFF);
    }
    LOG_GREEN_INFO("camera ", deviceIndex, " opened at ",
                   actualWidth, "x", actualHeight, " (", fourccText, ")");
    return true;
#endif
}

bool ArucoDetector::isCameraOpen() const {
    std::lock_guard<std::mutex> lock(captureMutex_);
#ifdef CAMERA_USE_LIBCAMERA
    return libcamera_ && libcamera_->isOpen();
#else
    return capture_.isOpened();
#endif
}

void ArucoDetector::releaseCamera() {
    std::lock_guard<std::mutex> lock(captureMutex_);
#ifdef CAMERA_USE_LIBCAMERA
    if (libcamera_) {
        libcamera_->close();
        libcamera_.reset();
    }
#else
    if (capture_.isOpened()) {
        capture_.release();
    }
#endif
}

bool ArucoDetector::captureFrame(cv::Mat& outFrame) {
    std::lock_guard<std::mutex> lock(captureMutex_);

#ifdef CAMERA_USE_LIBCAMERA
    if (!libcamera_ || !libcamera_->isOpen()) {
        return false;
    }

    RawFrame raw;
    if (!libcamera_->captureFrame(raw) || !convertToGray(raw, outFrame)) {
        outFrame.release();
        reportCaptureFailure(captureFailCount_);
        ++captureFailCount_;
        return false;
    }
#else
    if (!capture_.isOpened()) {
        return false;
    }

    capture_ >> outFrame;
    if (outFrame.empty()) {
        outFrame.release();
        reportCaptureFailure(captureFailCount_);
        ++captureFailCount_;
        return false;
    }

    // The pipeline is grayscale from here on; V4L2 webcams deliver BGR.
    if (outFrame.channels() == 3) {
        cv::cvtColor(outFrame, outFrame, cv::COLOR_BGR2GRAY);
    } else if (outFrame.channels() == 4) {
        cv::cvtColor(outFrame, outFrame, cv::COLOR_BGRA2GRAY);
    }
#endif

    if (captureFailCount_ > 0) {
        LOG_GREEN_INFO("camera delivered a frame after ",
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
    if (frame.channels() != 1) {
        LOG_ERROR("ArucoDetector::detect expects a grayscale frame, got ",
                  frame.channels(), " channels");
        return results;
    }

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> ids;

#if ARUCO_OPENCV_NEW_API
    detector_->detectMarkers(frame, markerCorners, ids);
#else
    cv::aruco::detectMarkers(frame, dictionary_, markerCorners, ids, parameters_);
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
