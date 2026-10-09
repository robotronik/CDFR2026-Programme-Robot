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

// Lists the real feature captures, tests/data/features/<n>.jpg. They come from
// the robot's camera and carry no ground-truth pose.
std::vector<std::string> featureCaptures() {
    const std::string prefixes[] = {"tests/data/features/", "../tests/data/features/",
                                    "data/features/"};
    for (const std::string& prefix : prefixes) {
        std::vector<std::string> names;
        cv::glob(prefix + "*.jpg", names, false);
        if (!names.empty()) {
            std::sort(names.begin(), names.end());
            return names;
        }
    }
    return {};
}

} // namespace

// The feature localiser must recover a field pose from a real capture by
// matching its rectified ground patch against the field map. The real frames
// carry neither a ground-truth pose nor a starting position, so the whole map is
// searched and any returned position counts. The test passes when at least one
// capture yields a position; every capture's outcome is logged.
bool test_features_localizer() {
    const std::string calibrationPath = findCalibrationPath("OV9281_1280_800.yaml");
    if (calibrationPath.empty()) {
        LOG_ERROR("Features test - missing OV9281 calibration");
        return false;
    }
    const std::string mapPath = findFeaturePath("table.png");
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
    for (const std::string& path : captures) {
        const std::string name = path.substr(path.find_last_of('/') + 1);
        const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
        if (image.empty()) {
            LOG_ERROR("Features test - cannot read ", path);
            return false;
        }

        vision::CameraPosition position;
        if (localizer.locate(image, position, /*hasStartingPosition=*/false)) {
            ++solved;
            LOG_INFO("Features test - ", name, ": position (", position.x, ", ",
                     position.y, ") mm, heading ", position.heading, " deg");
        } else {
            LOG_WARNING("Features test - ", name, ": no position found");
        }
    }

    LOG_INFO("Features test - found a position for ", solved, "/", captures.size(),
             " captures");
    if (solved == 0) {
        LOG_ERROR("Features test - no capture yielded a position");
        return false;
    }
    return true;
}
