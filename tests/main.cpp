#include <string>

#include "utils/logger.hpp"

bool testLogger();
bool test_lidar_opponent();
bool test_aruco_sim_camera();
bool test_aruco_localizer();
bool test_camera_robot_conversion();
bool test_game_element_cube_center();
bool test_features_localizer();
bool test_mat_parse();

int main(int argc, char** argv) {
    if (argc != 2) {
        LOG_ERROR("Usage: ", argv[0], " <log|lidar|mat|aruco|features>");
        return 1;
    }

    const std::string group = argv[1];
    bool ok = true;
    if (group == "log") {
        ok = testLogger();
    } else if (group == "lidar") {
        ok = test_lidar_opponent();
    } else if (group == "mat") {
        ok = test_mat_parse();
    } else if (group == "aruco") {
        ok &= test_aruco_sim_camera();
        ok &= test_aruco_localizer();
        ok &= test_camera_robot_conversion();
        ok &= test_game_element_cube_center();
    } else if (group == "features") {
        ok &= test_features_localizer();
    } else {
        LOG_ERROR("Unknown test '", group, "'. Expected log, lidar, mat, aruco or features.");
        return 1;
    }

    LOG_INFO("Test '", group, "' ", ok ? "PASSED" : "FAILED");
    return ok ? 0 : 1;
}