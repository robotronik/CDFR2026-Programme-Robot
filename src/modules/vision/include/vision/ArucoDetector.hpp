#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/videoio.hpp>

#include "vision/RawFrame.hpp"

#ifdef CAMERA_USE_LIBCAMERA
#  include "vision/LibcameraCamera.hpp"
#endif

// OpenCV 4.7 moved ArUco into `objdetect` (cv::aruco::ArucoDetector); older
// releases (e.g. on the Raspberry Pi) expose the legacy free functions.
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#  define ARUCO_OPENCV_NEW_API 1
#  include <opencv2/objdetect/aruco_detector.hpp>
#else
#  define ARUCO_OPENCV_NEW_API 0
#  include <opencv2/aruco.hpp>
#endif

namespace vision {

// One detected ArUco marker. `rvec`/`tvec` are set only when `hasPose` is true,
// which needs a loaded calibration and a registered size for the id (mm).
struct DetectionResult {
    int id = -1;
    std::vector<cv::Point2f> corners;
    cv::Vec3d rvec{0.0, 0.0, 0.0};
    cv::Vec3d tvec{0.0, 0.0, 0.0};
    bool hasPose = false;
};

// ArUco marker detector. Owns the capture device, the marker dictionary and the
// camera calibration; open a camera, then detect markers on a frame.
class ArucoDetector {
public:
    ArucoDetector();
    ~ArucoDetector();

    ArucoDetector(const ArucoDetector&) = delete;
    ArucoDetector& operator=(const ArucoDetector&) = delete;

    // Loads camera_matrix and dist_coeffs from an OpenCV YAML/XML file.
    bool loadCalibration(const std::string& calibrationFilePath);

    // Physical marker size in calibration units (mm). Pose is estimated only for
    // ids with a registered size.
    void setMarkerSize(int id, double size);

    bool initCamera(int deviceIndex = 0, int width = 640, int height = 480);
    bool isCameraOpen() const;
    void releaseCamera();

    // Thread-safe frame grab. Returns false (and leaves `outFrame` empty) when
    // the camera is closed or the frame cannot be read.
    bool captureFrame(cv::Mat& outFrame);

    // Controls
    void setExposureValue(float ev);
    void setContrast(float contrast);
    void setBrightness(float brightness);
    float getExposureValue() const { return exposureValue_; }
    float getContrast() const { return contrast_; }
    float getBrightness() const { return brightness_; }

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

#ifdef CAMERA_USE_LIBCAMERA
    // On the Pi 5 the OV9281 sits behind the PiSP ISP, so the raw V4L2 node only
    // yields frames OpenCV cannot decode; libcamera drives the full pipeline.
    std::unique_ptr<LibcameraCamera> libcamera_;
#else
    cv::VideoCapture capture_;
#endif
    float exposureValue_ = 1.5f;
    float contrast_ = 1.3f;
    float brightness_ = 0.0f;

    mutable std::mutex captureMutex_;
    // Consecutive failed frame reads, to report a camera that opened but never
    // delivers frames. Guarded by captureMutex_.
    int captureFailCount_ = 0;
};

} // namespace vision
