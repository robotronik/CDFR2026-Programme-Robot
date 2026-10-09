#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vision/Cam.hpp"
#include "vision/ArucoDetector.hpp"
#include "vision/ArucoLocalizer.hpp"
#include "utils/logger.hpp"

namespace {

// Virtual camera of Robotronik_CDFR_Sim_2027: 1280x800, 70 deg vertical FOV,
// mounted 233.2 mm above the ground and pitched down 45 deg.
constexpr double kSimCameraHeightMm = 233.2;

// The field tags are 100 mm squares lying on the ground.
constexpr double kTagSideMm = 100.0;

std::string findImagePath(const std::string& name) {
    const std::string candidates[] = {
        "tests/data/" + name,
        "../tests/data/" + name,
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

// Lists the game element captures, whose filenames encode the camera pose:
// capture_<n>_x<X>_y<Y>_yaw<A>_pitch<P>_vfov<V>_hfov<H>.png. A leading path
// prefix is tried so the tests run from either the build or the source tree.
std::vector<std::string> captureNames() {
    const std::string prefixes[] = {"tests/data/", "../tests/data/", ""};
    for (const std::string& prefix : prefixes) {
        std::vector<std::string> names;
        cv::glob(prefix + "capture_*.png", names, false);
        if (!names.empty()) {
            std::vector<std::string> basenames;
            for (const std::string& path : names) {
                basenames.push_back(path.substr(path.find_last_of('/') + 1));
            }
            std::sort(basenames.begin(), basenames.end());
            return basenames;
        }
    }
    return {};
}

// Lists the real localisation captures, tests/data/aruco_loc/<n>.jpg, hosting
// every frame the detector must be able to localise from. A leading path prefix
// is tried so the tests run from either the build or the source tree.
std::vector<std::string> locatorCaptureNames() {
    const std::string prefixes[] = {"tests/data/", "../tests/data/", ""};
    for (const std::string& prefix : prefixes) {
        std::vector<std::string> names;
        cv::glob(prefix + "aruco_loc/*.jpg", names, false);
        if (!names.empty()) {
            std::vector<std::string> basenames;
            for (const std::string& path : names) {
                basenames.push_back(path.substr(path.find_last_of('/') + 1));
            }
            std::sort(basenames.begin(), basenames.end());
            return basenames;
        }
    }
    return {};
}

// Resolves one real localisation capture in tests/data/aruco_loc.
std::string findLocatorImage(const std::string& name) {
    const std::string candidates[] = {
        "tests/data/aruco_loc/" + name,
        "../tests/data/aruco_loc/" + name,
    };
    for (const std::string& path : candidates) {
        if (!cv::imread(path, cv::IMREAD_GRAYSCALE).empty()) {
            return path;
        }
    }
    return {};
}

// Reads the camera pose from a capture filename. Returns false when the name
// does not carry the expected fields.
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

// Runs detection and pose estimation on captures from the virtual camera of
// Robotronik_CDFR_Sim_2027, using that camera's synthetic calibration. For each
// capture the filename records the camera's true field pose and the tag id is
// known, so the camera position in the marker frame can be computed twice and
// the two compared:
//   - detected: -R^T * tvec, the ArUco pose convention;
//   - expected: the camera's offset from the tag, from the true field poses.
bool test_aruco_sim_camera() {
    struct SampleCase {
        const char* image;
        int expectedId;
        double cameraFieldX; // camera (robot) field pose, from the filename
        double cameraFieldY;
        double tagFieldX;    // tag's known field position (SIM geometry)
        double tagFieldY;
    };
    // captures encode their true pose: capture_<..>_x<cx>_y<cy>_...
    static const SampleCase kCases[] = {
        {"sim_capture_22_tag22.png", 22, 586.6, -810.1, 400.0, -900.0},
        {"sim_capture_82_tag21.png", 21, -310.9, 706.4, -400.0, 900.0},
    };

    const std::string calibrationPath = findCalibrationPath("SIM_VFOV70_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_ERROR("ArUco sim test - missing virtual camera calibration");
        return false;
    }

    vision::ArucoDetector detector;
    if (!detector.loadCalibration(calibrationPath)) {
        LOG_ERROR("ArUco sim test - failed to load ", calibrationPath);
        return false;
    }
    for (int id : {20, 21, 22, 23}) {
        detector.setMarkerSize(id, kTagSideMm);
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

        // Camera position in the marker frame, from the detected pose.
        cv::Matx33d rotation;
        cv::Rodrigues(marker->rvec, rotation);
        const cv::Vec3d detected = -(rotation.t() * marker->tvec);

        // Same position from the true field poses: the camera (cy - ty, tx - cx)
        // relative to the tag, one camera height above the ground plane.
        const cv::Vec3d expected{testCase.cameraFieldY - testCase.tagFieldY,
                                 testCase.tagFieldX - testCase.cameraFieldX,
                                 kSimCameraHeightMm};

        const cv::Vec3d delta = detected - expected;
        const double errorMm = std::sqrt(delta.dot(delta));

        LOG_INFO("ArUco sim test - tag ", testCase.expectedId, " in ", testCase.image,
                 ": camera in marker frame detected (", detected[0], ", ", detected[1], ", ", detected[2],
                 ") mm, expected (", expected[0], ", ", expected[1], ", ", expected[2],
                 ") mm, error = ", errorMm, " mm");

        if (errorMm > 15.0) {
            LOG_ERROR("ArUco sim test - tag ", testCase.expectedId,
                      " camera position error too large: ", errorMm, " mm");
            return false;
        }
    }

    // A blank frame must not yield any detection (no false positives).
    const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(255));
    if (!detector.detect(blank).empty()) {
        LOG_ERROR("ArUco sim test - unexpected detection on a blank frame");
        return false;
    }

    return true;
}

// ArucoLocalizer wraps the detector and reports the camera's field position from
// the known landmark tags, or says it could not find one. The captures in
// tests/data/aruco_loc come from the robot's real OV9281 camera (1280x800) and
// are printed with very low contrast, so every one of them must still yield a
// landmark tag and a camera fix.
//
// The camera is bolted to the robot, so the height it reports from the tag poses
// is the same in every capture. A frame whose fix falls outside the accepted
// band is a wrong pose (a tag seen at a very oblique angle is the usual cause),
// not a different mounting, so it fails the test.
bool test_aruco_localizer() {
    const std::string calibrationPath = findCalibrationPath("OV9281_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_ERROR("ArUco localizer test - missing OV9281 calibration");
        return false;
    }

    const std::vector<std::string> captures = locatorCaptureNames();
    if (captures.empty()) {
        LOG_ERROR("ArUco localizer test - no capture found in tests/data/aruco_loc");
        return false;
    }

    vision::ArucoLocalizer localizer;
    if (!localizer.loadCalibration(calibrationPath)) {
        LOG_ERROR("ArUco localizer test - failed to load ", calibrationPath);
        return false;
    }

    // The height the tag poses report must match the camera mounting constant,
    // so the accepted band is centred on it.
    constexpr double kCameraHeightToleranceMm = 20.0;

    int located = 0;
    bool heightOk = true;
    for (const std::string& name : captures) {
        const std::string imagePath = findLocatorImage(name);
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

// The camera is mounted off the robot's centre, so the two frames differ.
// robotToCamera() and cameraToRobot() must be inverses of each other.
bool test_camera_robot_conversion() {
    struct SampleCase {
        double robotX;
        double robotY;
        double robotA;
    };
    static const SampleCase kCases[] = {
        {0.0, 0.0, 0.0},
        {500.0, -300.0, 90.0},
        {-250.0, 780.0, -135.0},
    };

    const double mountingDistance = std::hypot(OFFSET_CAM_X, OFFSET_CAM_Y);

    for (const SampleCase& testCase : kCases) {
        const position_t robot = {testCase.robotX, testCase.robotY, testCase.robotA};
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

// A game element is a tagged cube. gameElementFromTag() must report the cube's
// centre, not the centre of the marker on its visible face. The cube sits at
// (0, 0, 55) with no rotation in every capture; only the camera moves, so each
// visible marker must yield the same cube centre whatever the camera pose. Each
// capture records its camera pose in the filename.
bool test_game_element_cube_center() {
    const std::string calibrationPath = findCalibrationPath("SIM_VFOV70_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_ERROR("Game element test - missing virtual camera calibration");
        return false;
    }

    const std::vector<std::string> captures = captureNames();
    if (captures.empty()) {
        LOG_ERROR("Game element test - no capture found in tests/data");
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

        const std::string imagePath = findImagePath(name);
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
