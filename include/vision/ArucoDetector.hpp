#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/videoio.hpp>

// OpenCV moved the ArUco module into `objdetect` and introduced the
// cv::aruco::ArucoDetector class in 4.7. Older releases (including the one
// shipped on the Raspberry Pi) expose the legacy free functions from the
// `aruco` contrib module instead.
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#  define ARUCO_OPENCV_NEW_API 1
#  include <opencv2/objdetect/aruco_detector.hpp>
#else
#  define ARUCO_OPENCV_NEW_API 0
#  include <opencv2/aruco.hpp>
#endif

namespace vision {

/**
 * Result of a single detected ArUco marker.
 *
 * `rvec` / `tvec` are only meaningful when `hasPose` is true. Pose estimation
 * requires both a loaded camera calibration and a registered physical size for
 * the marker id. The translation is expressed in the same unit as the
 * calibration and the registered marker size (millimetres in this project).
 */
struct DetectionResult {
    int id = -1;
    std::vector<cv::Point2f> corners;
    cv::Vec3d rvec{0.0, 0.0, 0.0};
    cv::Vec3d tvec{0.0, 0.0, 0.0};
    bool hasPose = false;
};

/**
 * Native ArUco marker detector.
 *
 * Replaces the previous Python/REST camera service. It owns the capture device,
 * the marker dictionary/parameters and the camera calibration, and exposes a
 * small synchronous API: open a camera, then detect markers on a frame.
 */
class ArucoDetector {
public:
    ArucoDetector();
    ~ArucoDetector();

    ArucoDetector(const ArucoDetector&) = delete;
    ArucoDetector& operator=(const ArucoDetector&) = delete;

    // Loads `camera_matrix` and `dist_coeffs` from an OpenCV YAML/XML file.
    bool loadCalibration(const std::string& calibrationFilePath);

    // Physical marker size (same unit as the calibration, e.g. mm). Pose is only
    // estimated for ids that have a registered size.
    void setMarkerSize(int id, double size);

    bool initCamera(int deviceIndex = 0, int width = 640, int height = 480);
    bool isCameraOpen() const;
    void releaseCamera();

    // Thread-safe frame grab. Returns false (and leaves `outFrame` empty) when
    // the camera is closed or the frame cannot be read, without throwing.
    bool captureFrame(cv::Mat& outFrame);

    // Detects markers on an already acquired frame (BGR, BGRA or grayscale).
    std::vector<DetectionResult> detect(const cv::Mat& frame);

private:
#if ARUCO_OPENCV_NEW_API
    cv::Ptr<cv::aruco::ArucoDetector> detector_;
#else
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
#endif

    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    bool calibrated_ = false;

    std::map<int, double> markerSizes_;

    cv::VideoCapture capture_;
    mutable std::mutex captureMutex_;
};

} // namespace vision
