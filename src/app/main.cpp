#include <signal.h>
#include <thread>
#include <unistd.h>  // for usleep

#include "actions/Robot.hpp"
#include "utils/logger.hpp"
#include "utils/debugSound.hpp"

// Navigation
#include "navigation/navigation.h"
#include "navigation/pathfind.h"
#include "actions/calibration.h"

#include "utils/utils.h"
#include "restAPI/restAPI.hpp"
#include "restAPI/manual_mode.h"

// Actions
#include "actions/MainActionFSM.hpp"

// Arduino
#include "i2c/Arduino.hpp"
#include "actions/SensorControl.hpp"

#ifndef __CROSS_COMPILE_ARM__
    // Desktop Build
    #define DISABLE_LIDAR
    #define TEST_API_ONLY
    #define EMULATE_I2C
#endif

// Robot: owns the hardware (Arduino, lidar, camera, mat, sensors) and the
// chassis/actuator control (see actions/Robot.hpp).
Robot robot;
TableState tableStatus(&robot.colorTeam);

// The action FSM runs the selected strategy; it depends on the robot and table.
// (The robot's table reference + navigation are wired in StartSequence.)
ActionFSM action(&robot, &tableStatus);

MainState currentState;
MainState nextState;
bool initState;

std::thread api_server_thread;

// REST API
RestAPI api(&currentState, &nextState, &robot);

// Prototypes
int StartSequence();
void EndSequence();
void tests();

// Signal Management
bool exit_requested = false;
void ctrlc(int)
{
    LOG_INFO("Stop Signal Received");
    exit_requested = true;
}

int main()
{
    LOG_DEBUG("LOG ID : ", log_main_get_id());
    if (StartSequence() != 0)
        return -1;

    while (!exit_requested)
    {
        const unsigned long loopStartTime = _millis();

        // Refresh the sensors (drive, camera prior, lidar, mat).
        robot.update();

        // State machine
        switch (currentState)
        {
        //****************************************************************
        case MainState::Init:
        {
            if (initState)
            {
                LOG_GREEN_INFO("INIT");
                robot.actuators.disableActuators();
                robot.markHardwareStopped();
                robot.tableState().reset();
                robot.arduino()->RGB_Rainbow();
            }

            robot.updateTeamStrat();

            if (robot.sensor()->readButtonSensor() && !robot.sensor()->readLatchSensor() && robot.colorTeam != ColorTeam::None && robot.currentStrategy != nullptr)
                nextState = MainState::Calibration;
            break;
        }
        //****************************************************************
        case MainState::Calibration:
        {
            static bool has_calib = false;

            if (initState){
                LOG_GREEN_INFO("CALIBRATION");
                robot.startMatchHardware();
                robot.tableState().pos_opponent = OPPONENT_POS_UNKNOWN;
                robot.calibrationAge = -1; // Force a fresh camera localisation
            }

            if (manual_ctrl){
                // Debug escape hatch: allow driving without ever seeing an aruco tag.
                if (robot.calibrationAge == -1 && robot.colorTeam != ColorTeam::None) {
                    const position_t home = robot.currentStrategy->StratStartingPos();
                    robot.drive.setCoordinates(home);
                    robot.navigation().goTo(home);
                    LOG_INFO("Manual mode before calibration: assuming home position");
                }
                nextState = MainState::Manual;
                break;
            }

            if (robot.calibrationAge == -1) {
                // Get a camera fix; the OTOS calibration compares its moves against it.
                // Navigation only tries once per call and reports Done either way, so keep
                // asking until the fix lands (which resets the calibration age).
                robot.navigation().go();
            } else if (!has_calib) {
                // Move to the calibration poses, then back to the strategy starting position
                if (calibrate_otos(&robot.tableState(), &robot, robot.cam(), robot.currentStrategy->StratStartingPos())) {
                    LOG_GREEN_INFO("Calibration successful");
                    has_calib = true;
                }
            } else {
                nextState = MainState::WaitStart;
            }
            break;
        }
        //****************************************************************
        case MainState::WaitStart:
        {
            if (initState){
                LOG_GREEN_INFO("WAITSTART");
                // Already up when coming from Calibration, but a WaitStart forced through
                // the REST API skips calibration, so make sure the hardware is ready.
                robot.startMatchHardware();
            }

            // Calibrated and at the starting position: wait for the latch to start the match
            if (robot.sensor()->readLatchSensor())
                nextState = MainState::Run;
            if (manual_ctrl)
                nextState = MainState::Manual;
            break;
        }
        //****************************************************************
        case MainState::Run:
        {
            if (initState){
                LOG_GREEN_INFO("RUN");
                robot.tableState().reset();
                robot.tableState().startTime = _millis();
                action.Reset();
            }
            bool finished = action.RunFSM();

            if (_millis() > robot.tableState().startTime + MATCH_DURATION_MS || finished || robot.sensor()->readButtonSensor())
                nextState = MainState::Fin;
            break;
        }
        //****************************************************************
        case MainState::Test:
        {
            if (initState){
                LOG_GREEN_INFO("TEST");
            }

            tests();
            break;
        }
        //****************************************************************
        case MainState::Manual:
        {
            if (initState){
                LOG_GREEN_INFO("MANUAL");
                robot.arduino()->RGB_Blinking(255, 0, 255); // Purple blinking
            }
            robot.navigation().go();

            // Execute the function as long as it returns false
            manual_loop();
            if (!manual_ctrl){
                exit_requested = true;
            }
            break;
        }
        //****************************************************************
        case MainState::Fin:
        {
            if (initState){
                LOG_GREEN_INFO("FIN");
                debugSound::play("end.wav");
                robot.arduino()->RGB_Solid(0, 255, 0);
                manual_clearFunc();
                robot.stopMatchHardware();
            }

            // Wait for the operator to release the latch, then quit
            if (!robot.sensor()->readLatchSensor()) {
                exit_requested = true;
            }
            break;
        }
        } // switch (currentState)

        initState = false;
        if (currentState != nextState)
        {
            initState = true;
            currentState = nextState;
        }

        // Check if state machine is running above loop time
        unsigned long ms = _millis();
        if (ms > loopStartTime + LOOP_TIME_MS){
            LOG_WARNING("Loop took more than " , LOOP_TIME_MS, "ms to execute (", (ms - loopStartTime), " ms)");
        }
        //State machine runs at a constant rate
        while (_millis() < loopStartTime + LOOP_TIME_MS){
            usleep(100);
        }
    }

    EndSequence();
    return 0;
}

int StartSequence()
{
    signal(SIGTERM, ctrlc);
    signal(SIGINT, ctrlc);

    setProgramPriority();

    if (!robot.startup())
        return -1;

    // Hand the table to the robot: builds navigation and the strategy pool.
    robot.setTableState(&tableStatus);

    // Start the api server in a separate thread
    api_server_thread = std::thread([&]()
                                    { api.start(); });

#ifdef TEST_API_ONLY
    LOG_GREEN_INFO("Running in API test mode only");
    currentState = MainState::Test;
    nextState = MainState::Test;
#else
    currentState = MainState::Init;
    nextState = MainState::Init;
#endif // TEST_API_ONLY

    // Wire the collaborators created outside the robot.
    robot.setCurrentState(&currentState);

    initState = true;
    manual_init();

    pathfind_setup(robot.tableState());

    LOG_GREEN_INFO("Init sequence done");
    debugSound::play("start.wav");
    return 0;
}

void EndSequence()
{
    LOG_GREEN_INFO("Stopping");

    robot.shutdown();

    // Stop the API server
    api.stop();
    api_server_thread.join();

    debugSound::play("shutdown.wav");
    sleep(1); // Let the sound start: the process is about to exit.
    LOG_GREEN_INFO("Stopped");
}


void tests()
{
    /*
        * Call test functions here.
        * They will be executed in loop until the program is stopped.
    */
}