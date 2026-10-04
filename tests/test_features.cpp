#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "utils/logger.hpp"
#include "vision/CamLocalizer.hpp"
#include "vision/FeaturesLocalizer.hpp"

namespace {

// Virtual camera of Robotronik_CDFR_Sim_2027, which the feature captures were
// rendered with: 1280x800, 70 deg vertical FOV, 233.2 mm above the ground,
// pitched down 45 deg.
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

std::string findFeaturePath(const std::string& name) {
    const std::string candidates[] = {
        "tests/data/features/" + name,
        "../tests/data/features/" + name,
        "data/features/" + name,
    };
    for (const std::string& path : candidates) {
        if (!cv::imread(path, cv::IMREAD_GRAYSCALE).empty()) {
            return path;
        }
    }
    return {};
}

// Lists the feature captures, whose filenames encode the camera's true pose:
// capture_<n>_x<X>_y<Y>_yaw<A>_pitch45.0.png.
std::vector<std::string> featureCaptures() {
    const std::string prefixes[] = {"tests/data/features/", "../tests/data/features/",
                                    "data/features/"};
    for (const std::string& prefix : prefixes) {
        std::vector<std::string> names;
        cv::glob(prefix + "capture_*.png", names, false);
        if (!names.empty()) {
            std::sort(names.begin(), names.end());
            return names;
        }
    }
    return {};
}

bool parseCapturePose(const std::string& path, vision::CameraPosition& pose) {
    const std::string name = path.substr(path.find_last_of('/') + 1);
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
    pose.x = std::stod(x);
    pose.y = std::stod(y);
    pose.heading = std::stod(yaw);
    pose.z = CAMERA_HEIGHT_MM;
    return true;
}

double poseErrorMm(const vision::CameraPosition& a, const vision::CameraPosition& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double headingErrorDeg(double a, double b) {
    return std::fabs(std::remainder(a - b, 360.0));
}

} // namespace

// The feature localiser must recover the camera's field pose from a capture by
// matching its rectified ground patch against the field map. Each capture
// records its true pose in the filename, so the solve can be checked against it.
// The odometry prior is the truth here, which is the best case for the prior
// window; a run without any prior is checked separately.
bool test_features_localizer() {
    const std::string calibrationPath = findCalibrationPath("SIM_VFOV70_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_ERROR("Features test - missing virtual camera calibration");
        return false;
    }
    const std::string mapPath = findFeaturePath("FieldBW.png");
    if (mapPath.empty()) {
        LOG_ERROR("Features test - missing field map");
        return false;
    }

    const std::vector<std::string> captures = featureCaptures();
    if (captures.empty()) {
        LOG_ERROR("Features test - no capture found in tests/data/features");
        return false;
    }

    vision::FeaturesLocalizer localizer;
    if (!localizer.loadCalibration(calibrationPath)) {
        LOG_ERROR("Features test - failed to load ", calibrationPath);
        return false;
    }
    if (!localizer.loadMap(mapPath)) {
        LOG_ERROR("Features test - failed to load ", mapPath);
        return false;
    }

    int solved = 0;
    double worstError = 0.0;
    double worstHeading = 0.0;
    for (const std::string& path : captures) {
        const std::string name = path.substr(path.find_last_of('/') + 1);
        vision::CameraPosition truth;
        if (!parseCapturePose(path, truth)) {
            LOG_ERROR("Features test - cannot read pose from ", name);
            return false;
        }

        const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
        if (image.empty()) {
            LOG_ERROR("Features test - cannot read ", path);
            return false;
        }

        localizer.setPrior(truth);
        vision::CameraPosition position;
        if (!localizer.locate(image, position)) {
            LOG_WARNING("Features test - no fix for ", name);
            continue;
        }

        const double errorMm = poseErrorMm(position, truth);
        const double errorDeg = headingErrorDeg(position.heading, truth.heading);
        worstError = std::max(worstError, errorMm);
        worstHeading = std::max(worstHeading, errorDeg);
        ++solved;

        LOG_INFO("Features test - ", name, ": camera (", position.x, ", ", position.y,
                 ") mm, heading ", position.heading, " deg | expected (", truth.x, ", ",
                 truth.y, ") mm, heading ", truth.heading, " deg | error ", errorMm,
                 " mm, ", errorDeg, " deg");
    }

    LOG_INFO("Features test - solved ", solved, "/", captures.size(),
             ", worst position error ", worstError, " mm, worst heading error ",
             worstHeading, " deg");

    if (solved < static_cast<int>(captures.size())) {
        LOG_ERROR("Features test - some captures were not solved");
        return false;
    }
    if (worstError > 60.0) {
        LOG_ERROR("Features test - position error too large: ", worstError, " mm");
        return false;
    }
    if (worstHeading > 15.0) {
        LOG_ERROR("Features test - heading error too large: ", worstHeading, " deg");
        return false;
    }
    return true;
}

// Without an odometry prior the whole map is offered to the matcher. It must
// still solve, just with more candidates to sift through.
bool test_features_localizer_without_prior() {
    const std::string calibrationPath = findCalibrationPath("SIM_VFOV70_1280_800.yaml");
    const std::string mapPath = findFeaturePath("FieldBW.png");
    if (calibrationPath.empty() || mapPath.empty()) {
        LOG_ERROR("Features test - missing calibration or map");
        return false;
    }

    const std::vector<std::string> captures = featureCaptures();
    if (captures.empty()) {
        LOG_ERROR("Features test - no capture found");
        return false;
    }

    vision::FeaturesLocalizer localizer;
    if (!localizer.loadCalibration(calibrationPath) || !localizer.loadMap(mapPath)) {
        LOG_ERROR("Features test - failed to initialise localiser");
        return false;
    }
    localizer.clearPrior();

    int solved = 0;
    for (const std::string& path : captures) {
        vision::CameraPosition truth;
        if (!parseCapturePose(path, truth)) {
            continue;
        }
        const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
        vision::CameraPosition position;
        if (localizer.locate(image, position)) {
            ++solved;
        }
    }

    LOG_INFO("Features test (no prior) - solved ", solved, "/", captures.size());
    if (solved < static_cast<int>(captures.size())) {
        LOG_ERROR("Features test - some captures were not solved without a prior");
        return false;
    }
    return true;
}
