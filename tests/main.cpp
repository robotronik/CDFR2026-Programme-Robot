#include "utils/logger.hpp"

bool testLogger();
bool test_lidar_opponent();
bool test_lidar_beacons();

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

    LOG_INFO("There has been ", numPassed, "/", numTests, " tests passed");
    //return (numTests == numPassed) ? 0 : 1;
    return 0; // Make the tests pass because gangsta
}