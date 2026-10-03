#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vision/ArucoDetector.hpp"
#include "utils/logger.hpp"

namespace {

std::string findImagePath(const std::string& name) {
    const std::string candidates[] = {
        "tests/data/" + name,
        "../tests/data/" + name,
        "data/" + name,
        "../data/" + name,
        name,
    };
    for (const std::string& path : candidates) {
        if (!cv::imread(path, cv::IMREAD_GRAYSCALE).empty()) {
            return path;
        }
    }
    return {};
}

std::string findCalibrationPath(const std::string& name) {
    const std::string candidates[] = {
        "data/" + name,
        "../data/" + name,
        "tests/data/" + name,
    };
    for (const std::string& path : candidates) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            return path;
        }
    }
    return {};
}

double polygonArea(const std::vector<cv::Point2f>& corners) {
    double area = 0.0;
    for (size_t i = 0; i < corners.size(); ++i) {
        const cv::Point2f& a = corners[i];
        const cv::Point2f& b = corners[(i + 1) % corners.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return std::fabs(area) / 2.0;
}

const vision::DetectionResult* findMarker(const std::vector<vision::DetectionResult>& detections, int id) {
    for (const auto& detection : detections) {
        if (detection.id == id) {
            return &detection;
        }
    }
    return nullptr;
}

} // namespace

// Loads the static synthetic image with marker id 33 and checks that detect()
// reports the expected id and four corners.
bool test_aruco_detection() {
    const std::string imagePath = findImagePath("aruco_marker_33.png");
    if (imagePath.empty()) {
        LOG_ERROR("ArUco test - sample image not found");
        return false;
    }

    const cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        LOG_ERROR("ArUco test - failed to load ", imagePath);
        return false;
    }

    vision::ArucoDetector detector;
    detector.setMarkerSize(33, 100.0);

    const std::vector<vision::DetectionResult> detections = detector.detect(image);
    const vision::DetectionResult* marker = findMarker(detections, 33);
    if (marker == nullptr) {
        LOG_ERROR("ArUco test - marker 33 was not detected in ", imagePath);
        return false;
    }
    if (marker->corners.size() != 4) {
        LOG_ERROR("ArUco test - marker 33 did not return 4 corners");
        return false;
    }
    if (polygonArea(marker->corners) < 100.0) {
        LOG_ERROR("ArUco test - marker 33 corners are degenerate");
        return false;
    }

    // A blank frame must not yield any detection (no false positives).
    const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(255));
    if (!detector.detect(blank).empty()) {
        LOG_ERROR("ArUco test - unexpected detection on a blank frame");
        return false;
    }

    return true;
}

// Checks that loading a calibration and registering a marker size enables pose
// estimation for the detected marker.
bool test_aruco_pose() {
    const std::string imagePath = findImagePath("aruco_marker_33.png");
    const std::string calibrationPath = findCalibrationPath("OV9281_1280_800.yaml");
    if (imagePath.empty() || calibrationPath.empty()) {
        LOG_WARNING("ArUco pose test skipped (missing sample image or calibration)");
        return true;
    }

    const cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    vision::ArucoDetector detector;
    if (!detector.loadCalibration(calibrationPath) || image.empty()) {
        LOG_ERROR("ArUco pose test - setup failed");
        return false;
    }
    detector.setMarkerSize(33, 100.0);

    const std::vector<vision::DetectionResult> detections = detector.detect(image);
    const vision::DetectionResult* marker = findMarker(detections, 33);
    if (marker == nullptr || !marker->hasPose) {
        LOG_ERROR("ArUco pose test - marker 33 pose was not estimated");
        return false;
    }

    const cv::Vec3d& r = marker->rvec;
    const cv::Vec3d& t = marker->tvec;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(r[i]) || !std::isfinite(t[i])) {
            LOG_ERROR("ArUco pose test - non-finite pose values");
            return false;
        }
    }
    if (t[2] <= 0.0) {
        LOG_ERROR("ArUco pose test - marker should be in front of the camera (z > 0)");
        return false;
    }

    return true;
}

// Runs detection and pose estimation on captures from the virtual camera of
// Robotronik_CDFR_Sim_2027, using the camera's synthetic calibration. The tag
// id and the camera's true field pose (mm, yaw in degrees) for each capture
// come from the simulation's own filenames:
//   capture_22_x586.6_y-810.1_yaw-109.2_pitch45.0 -> tag 22
//   capture_82_x-310.9_y706.4_yaw-236.9_pitch45.0 -> tag 21
bool test_aruco_sim_camera() {
    struct SampleCase {
        const char* image;
        int expectedId;
        double truthX;
        double truthY;
        double truthYaw;
    };
    static const SampleCase kCases[] = {
        {"sim_capture_22_tag22.png", 22, 586.6, -810.1, -109.2},
        {"sim_capture_82_tag21.png", 21, -310.9, 706.4, -236.9},
    };

    const std::string calibrationPath = findCalibrationPath("SIM_VFOV70_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_WARNING("ArUco sim test skipped (missing virtual camera calibration)");
        return true;
    }

    vision::ArucoDetector detector;
    if (!detector.loadCalibration(calibrationPath)) {
        LOG_ERROR("ArUco sim test - failed to load ", calibrationPath);
        return false;
    }
    for (int id : {20, 21, 22, 23}) {
        detector.setMarkerSize(id, 100.0); // the field tags are 100 mm on the ground
    }

    for (const SampleCase& testCase : kCases) {
        const std::string imagePath = findImagePath(testCase.image);
        if (imagePath.empty()) {
            LOG_ERROR("ArUco sim test - missing capture ", testCase.image);
            return false;
        }

        const cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
        const std::vector<vision::DetectionResult> detections = detector.detect(image);
        const vision::DetectionResult* marker = findMarker(detections, testCase.expectedId);
        if (marker == nullptr) {
            LOG_ERROR("ArUco sim test - tag ", testCase.expectedId, " not detected in ", imagePath);
            return false;
        }
        if (marker->corners.size() != 4 || polygonArea(marker->corners) < 100.0) {
            LOG_ERROR("ArUco sim test - tag ", testCase.expectedId, " corners are invalid");
            return false;
        }
        if (!marker->hasPose || !std::isfinite(marker->tvec[2]) || marker->tvec[2] <= 0.0) {
            LOG_ERROR("ArUco sim test - tag ", testCase.expectedId, " pose is invalid");
            return false;
        }

        LOG_INFO("ArUco sim test - tag ", testCase.expectedId, " in ", testCase.image,
                 " : aruco position (x, y, z) = (",
                 marker->tvec[0], ", ", marker->tvec[1], ", ", marker->tvec[2], ") mm",
                 " rvec = (", marker->rvec[0], ", ", marker->rvec[1], ", ", marker->rvec[2], ")",
                 " | real position x = ", testCase.truthX, " mm, y = ", testCase.truthY,
                 " mm, yaw = ", testCase.truthYaw, " deg");
    }

    return true;
}
