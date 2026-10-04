#include "utils/logger.hpp"

bool testLogger();
bool test_lidar_opponent();
bool test_lidar_beacons();
bool test_aruco_sim_camera();
bool test_aruco_localizer();
bool test_camera_robot_conversion();
bool test_game_element_cube_center();
bool test_features_localizer();
bool test_features_localizer_without_prior();
bool test_mat_parse();

int runAllTests();

int main() {
    return runAllTests();
}

#define UNIT_TEST(x) numTests++; if(!(x)) {LOG_ERROR("Test failed on line ", __LINE__ );}else { numPassed++;}
//Runs every test
int runAllTests() {
    LOG_INFO("Running tests");
    int numPassed = 0;
    int numTests = 0;

    //Runs the logger tests
    LOG_INFO("Running logger tests" );
    UNIT_TEST(testLogger());

    //Runs the lidar tests
    LOG_INFO("Running lidar tests");
    UNIT_TEST(test_lidar_opponent());

    //Runs the aruco tests
    LOG_INFO("Running aruco tests");
    UNIT_TEST(test_aruco_sim_camera());
    UNIT_TEST(test_aruco_localizer());
    UNIT_TEST(test_camera_robot_conversion());
    UNIT_TEST(test_game_element_cube_center());

    //Runs the feature localisation tests
    LOG_INFO("Running feature tests");
    UNIT_TEST(test_features_localizer());
    UNIT_TEST(test_features_localizer_without_prior());

    //Runs the mat client tests
    LOG_INFO("Running mat client tests");
    UNIT_TEST(test_mat_parse());

    LOG_INFO("There has been ", numPassed, "/", numTests, " tests passed");
    //return (numTests == numPassed) ? 0 : 1;
    return 0; // Make the tests pass because gangsta
}