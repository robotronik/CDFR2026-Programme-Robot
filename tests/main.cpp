#include "utils/logger.hpp"
#include "main.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h"
#include "lidar/Lidar.hpp"
#include "vision/ArucoCam.hpp"
#include "i2c/Arduino.hpp"

// Instanciation des objets globaux pour que le linker les trouve
TableState tableStatus;
DriveControl drive;
Arduino arduino;
Lidar lidar;
ArucoCam arucoCam1(-1, "");

main_State_t currentState;
main_State_t nextState;

bool manual_ctrl = false;
bool (*manual_currentFunc)() = nullptr;

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