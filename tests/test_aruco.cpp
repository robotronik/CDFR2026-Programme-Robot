#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vision/Cam.hpp"
#include "utils/logger.hpp"

namespace {

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

// Lists the capture basenames matching `name` under tests/data/<subdir>. A
// leading path prefix is tried so the tests run from either the build or source
// tree.
std::vector<std::string> captureBasenames(const std::string& subdir, const std::string& name) {
    const std::string prefixes[] = {"tests/data/", "../tests/data/", ""};
    for (const std::string& prefix : prefixes) {
        std::vector<std::string> paths;
        cv::glob(prefix + subdir + name, paths, false);
        if (paths.empty()) {
            continue;
        }
        std::vector<std::string> basenames;
        for (const std::string& path : paths) {
            basenames.push_back(path.substr(path.find_last_of('/') + 1));
        }
        std::sort(basenames.begin(), basenames.end());
        return basenames;
    }
    return {};
}

// Resolves one capture named `name` in tests/data/<subdir>.
std::string findCapture(const std::string& subdir, const std::string& name) {
    const std::string candidates[] = {
        "tests/data/" + subdir + name,
        "../tests/data/" + subdir + name,
    };
    for (const std::string& path : candidates) {
        if (!cv::imread(path, cv::IMREAD_GRAYSCALE).empty()) {
            return path;
        }
    }
    return {};
}

// Reads the camera pose from a capture filename; false when a field is missing.
bool parseCapturePose(const std::string& name, position_t& cameraPose) {
    const auto field = [&](const std::string& key) -> const char* {
        const size_t at = name.find(key);
        return at == std::string::npos ? nullptr : name.c_str() + at + key.size();
    };
    const char* x = field("_x");
    const char* y = field("_y");
    const char* yaw = field("_yaw");
    if (x == nullptr || y == nullptr || yaw == nullptr) {
        return false;
    }
    cameraPose.x = std::stod(x);
    cameraPose.y = std::stod(y);
    cameraPose.a = std::stod(yaw);
    return true;
}

} // namespace

// The real captures in tests/data/aruco_loc (OV9281, 1280x800) are low contrast,
// so every one must still yield a landmark tag and a camera fix. The camera is
// bolted to the robot, so the reported height is the same every frame; a fix
// outside the accepted band is a wrong pose, not a different mounting.
bool test_aruco_localizer() {
    const std::string calibrationPath = findCalibrationPath("OV9281_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_ERROR("ArUco localizer test - missing OV9281 calibration");
        return false;
    }

    const std::vector<std::string> captures = captureBasenames("aruco_loc/", "*.jpg");
    if (captures.empty()) {
        LOG_ERROR("ArUco localizer test - no capture found in tests/data/aruco_loc");
        return false;
    }

    vision::ArucoLocalizer localizer;
    if (!localizer.loadCalibration(calibrationPath)) {
        LOG_ERROR("ArUco localizer test - failed to load ", calibrationPath);
        return false;
    }

    // The accepted band is centred on the camera mounting height.
    constexpr double kCameraHeightToleranceMm = 20.0;

    int located = 0;
    bool heightOk = true;
    for (const std::string& name : captures) {
        const std::string imagePath = findCapture("aruco_loc/", name);
        if (imagePath.empty()) {
            LOG_ERROR("ArUco localizer test - missing capture ", name);
            return false;
        }

        const cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
        const std::vector<vision::DetectionResult> detections = localizer.detector().detect(image);

        // The landmark tags (ids 20..23) are the ones the localizer can use.
        const vision::DetectionResult* landmark = nullptr;
        for (const vision::DetectionResult& detection : detections) {
            if (vision::ArucoLocalizer::fieldPosition(detection.id) != nullptr) {
                landmark = &detection;
                break;
            }
        }
        if (landmark == nullptr) {
            LOG_ERROR("ArUco localizer test - no landmark tag detected in ", imagePath);
            return false;
        }

        vision::CameraPosition position;
        if (!localizer.locate(detections, position)) {
            LOG_ERROR("ArUco localizer test - no position found in ", imagePath);
            return false;
        }
        if (!std::isfinite(position.z) || position.z <= 0.0) {
            LOG_ERROR("ArUco localizer test - invalid camera height in ", imagePath);
            return false;
        }

        // Pitch is the angle of the camera's optical axis below the horizon: the
        // marker-frame height of its forward axis, negated.
        cv::Matx33d rotation;
        cv::Rodrigues(landmark->rvec, rotation);
        const cv::Vec3d viewAxis = rotation.t() * cv::Vec3d(0.0, 0.0, 1.0);
        const double pitchDeg = std::asin(std::clamp(-viewAxis[2], -1.0, 1.0)) * 180.0 / M_PI;

        LOG_INFO("ArUco localizer test - ", name, ": tag ", landmark->id,
                 ", camera (", position.x, ", ", position.y, ") mm, height ", position.z,
                 " mm, pitch ", pitchDeg, " deg, heading ", position.heading, " deg");

        if (std::fabs(position.z - CAMERA_HEIGHT_MM) > kCameraHeightToleranceMm) {
            LOG_ERROR("ArUco localizer test - ", name, ": camera height ", position.z,
                      " mm is outside [", CAMERA_HEIGHT_MM - kCameraHeightToleranceMm, ", ",
                      CAMERA_HEIGHT_MM + kCameraHeightToleranceMm, "] mm");
            heightOk = false;
        }
        ++located;
    }

    if (located != static_cast<int>(captures.size())) {
        LOG_ERROR("ArUco localizer test - only ", located, " of ", captures.size(),
                  " captures located");
        return false;
    }
    if (!heightOk) {
        LOG_ERROR("ArUco localizer test - the camera height is not fixed across all captures");
        return false;
    }

    // A frame with no known tag must report that no position was found.
    const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(255));
    vision::CameraPosition none;
    if (localizer.locate(blank, none)) {
        LOG_ERROR("ArUco localizer test - reported a position for a blank frame");
        return false;
    }

    return true;
}

// The camera is mounted off the robot's centre, so the two frames differ;
// robotToCamera() and cameraToRobot() must be inverses of each other.
bool test_camera_robot_conversion() {
    static const position_t kCases[] = {
        {0.0, 0.0, 0.0},
        {500.0, -300.0, 90.0},
        {-250.0, 780.0, -135.0},
    };

    const double mountingDistance = std::hypot(OFFSET_CAM_X, OFFSET_CAM_Y);

    for (const position_t& robot : kCases) {
        position_t camera = robotToCamera(robot);

        const double offset = std::hypot(camera.x - robot.x, camera.y - robot.y);
        if (std::fabs(offset - mountingDistance) > 1e-6) {
            LOG_ERROR("Camera/robot conversion test - camera offset is ", offset,
                      " mm, expected ", mountingDistance, " mm");
            return false;
        }

        const position_t back = cameraToRobot(camera);
        const double error = std::hypot(back.x - robot.x, back.y - robot.y);
        if (error > 1e-6 || std::fabs(std::remainder(back.a - robot.a, 360.0)) > 1e-6) {
            LOG_ERROR("Camera/robot conversion test - round trip failed, error ", error, " mm");
            return false;
        }
    }

    return true;
}

// gameElementFromTag() must report the cube's centre, not the marker's. The cube
// sits at (0, 0, 55) with no rotation in every capture, so each visible marker
// must yield the same centre whatever the camera pose (encoded in the filename).
bool test_game_element_cube_center() {
    const std::string calibrationPath = findCalibrationPath("SIM_VFOV70_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_ERROR("Game element test - missing virtual camera calibration");
        return false;
    }

    const std::vector<std::string> captures = captureBasenames("aruco_blocs/", "capture_*.png");
    if (captures.empty()) {
        LOG_ERROR("Game element test - no capture found in tests/data/aruco_blocs");
        return false;
    }

    vision::ArucoDetector detector;
    if (!detector.loadCalibration(calibrationPath)) {
        LOG_ERROR("Game element test - failed to load ", calibrationPath);
        return false;
    }
    detector.setMarkerSize(13, GAME_ELEMENT_TAG_MM);

    const double expectedX = 0.0, expectedY = 0.0, expectedZ = 55.0;

    for (const std::string& name : captures) {
        position_t cameraPose = {0.0, 0.0, 0.0};
        if (!parseCapturePose(name, cameraPose)) {
            LOG_ERROR("Game element test - cannot read camera pose from ", name);
            return false;
        }

        const std::string imagePath = findCapture("aruco_blocs/", name);
        if (imagePath.empty()) {
            LOG_ERROR("Game element test - missing capture ", name);
            return false;
        }

        const cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
        const std::vector<vision::DetectionResult> detections = detector.detect(image);

        // A field landmark tag may also be present but is not a game element.
        std::vector<GameElement> elements;
        for (const vision::DetectionResult& detection : detections) {
            GameElement element;
            if (Cam::gameElementFromTag(detection, cameraPose, element)) {
                elements.push_back(element);
            }
        }

        if (elements.empty()) {
            LOG_ERROR("Game element test - ", name, ": no game element found");
            return false;
        }

        // When more than one face is visible they must agree on the cube centre.
        for (size_t i = 0; i < elements.size(); ++i) {
            for (size_t j = i + 1; j < elements.size(); ++j) {
                const GameElement& a = elements[i];
                const GameElement& b = elements[j];
                const double spread = std::sqrt(std::pow(a.x - b.x, 2) +
                                                std::pow(a.y - b.y, 2) +
                                                std::pow(a.z - b.z, 2));
                if (spread > 15.0) {
                    LOG_ERROR("Game element test - ", name,
                              ": two visible tags disagree on the cube centre: ", spread, " mm");
                    return false;
                }
            }
        }

        const GameElement& a = elements[0];
        const double error = std::sqrt(std::pow(a.x - expectedX, 2) +
                                       std::pow(a.y - expectedY, 2) +
                                       std::pow(a.z - expectedZ, 2));
        LOG_INFO("Game element test - ", name, ": cube centre (", a.x, ", ", a.y, ", ",
                 a.z, ") mm, expected (", expectedX, ", ", expectedY, ", ", expectedZ,
                 ") mm, error = ", error, " mm, faces = ", elements.size());
        if (error > 15.0) {
            LOG_ERROR("Game element test - ", name,
                      ": cube centre error too large: ", error, " mm");
            return false;
        }
    }

    return true;
}
