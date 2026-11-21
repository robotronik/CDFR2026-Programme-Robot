#include "main.hpp"

#include <stdlib.h>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>  // for usleep

#include "actions/RobotControl.hpp"
#include "utils/logger.hpp"
#include "utils/debugSound.hpp"

// Lidar
#include "lidar/Lidar.hpp"
#include "lidar/lidarAnalize.h"

// Navigation
#include "navigation/navigation.h"
#include "navigation/pathfind.h"
#include "actions/calibration.h"

#include "utils/utils.h"
#include "restAPI/restAPI.hpp"
#include "restAPI/manual_mode.h"
#include "vision/Cam.hpp"
#include "mat/mat.hpp"

// Actions
#include "actions/MainActionFSM.hpp"
#include "actions/Strategy/ExempleStrat.hpp"

// Arduino
#include "i2c/Arduino.hpp"
#include "actions/SensorControl.hpp"

#ifndef __CROSS_COMPILE_ARM__
    // Desktop Build
    #define DISABLE_LIDAR
    #define TEST_API_ONLY
    #define EMULATE_I2C
#endif

// Hardware init
Arduino arduino;
Lidar lidar = Lidar(&arduino);

// control of hardware
SensorControl sensor(&arduino);
RobotControl robot(&arduino);
// status of table
TableState tableStatus(&robot);

// Brain Init
VirtualStrategy* currentStrategy = new ExempleStrat(&tableStatus, &robot);

// Strategies selectable through the REST API
std::vector<VirtualStrategy*> strategies = { currentStrategy };

ActionFSM action(currentStrategy, &robot, &tableStatus);

#ifndef EMULATE_CAM
Cam cam1 = Cam(0, "data/OV9281_1280_800.yaml", "data/FieldBW.png");
#else
Cam cam1(-1, "", "");
#endif

// Navigation
Navigation navigation(&robot.drive, &tableStatus, &cam1);

MainState currentState;
MainState nextState;
bool initState;

std::thread api_server_thread;

// REST API
RestAPI api(&currentState, &nextState, &robot, &tableStatus, &arduino, &sensor, &lidar, &cam1, &navigation, &strategies);

// Prototypes
int StartSequence();
void GetLidar();
void GetMat();
void EndSequence();
void tests();

// Signal Management
bool exit_requested = false;
void ctrlc(int)
{
    LOG_INFO("Stop Signal Received");
    exit_requested = true;
}

// Bring the robot up for a match: actuators enabled and homed, lidar spinning, camera running.
// Idempotent; Init and Fin tear the hardware down and clear the flag.
// TODO: move this into class Robot (hardware bring-up does not belong in the app state machine).
static bool hw_ready = false;
static void startMatchHardware()
{
    if (hw_ready)
        return;
    robot.actuators.enableActuators();
    // The wheels start at the strategy's starting position: set both the
    // robot's odometry position and its target there, and initialise the
    // navigation target so the first move starts from the right pose.
    const position_t startPos = action.getStrategy()->StratStartingPos();
    robot.drive.setCoordinates(startPos);
    navigation.goTo(startPos);
    robot.actuators.homeActuators();
    lidar.startSpin();
    cam1.start();
    hw_ready = true;
}

int main()
{
    LOG_DEBUG("LOG ID : ", log_main_get_id());
    if (StartSequence() != 0)
        return -1;

    while (!exit_requested)
    {
        const unsigned long loopStartTime = _millis();

        // Get Sensor Data
        {
            robot.drive.update();
            // The odometry prior helps the feature localiser; the marker
            // localiser ignores it.
            cam1.setPrior(robot.drive.position);

            if (currentState != MainState::Init && currentState != MainState::Fin)
            {
#ifndef DISABLE_LIDAR
                GetLidar();
#endif
                GetMat();
            }
        }

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
                hw_ready = false;
                tableStatus.reset();
                arduino.RGB_Rainbow();
            }

            update_team_strat();

            if (sensor.readButtonSensor() && !sensor.readLatchSensor() && tableStatus.colorTeam != ColorTeam::None && currentStrategy != nullptr)
                nextState = MainState::Calibration;
            break;
        }
        //****************************************************************
        case MainState::Calibration:
        {
            static bool has_calib = false;

            if (initState){
                LOG_GREEN_INFO("CALIBRATION");
                startMatchHardware();
                tableStatus.pos_opponent = OPPONENT_POS_UNKNOWN;
                tableStatus.calibrationAge = -1; // Force a fresh camera localisation
            }

            if (tableStatus.calibrationAge == -1){
                // Get a camera fix; the OTOS calibration compares its moves against it.
                // Navigation only tries once per call and reports Done either way, so keep
                // asking until the fix lands (which resets the calibration age).
                navigation.go();
            } else if (!has_calib){
                // Move to the calibration poses, then back to the strategy starting position
                if (calibrate_otos(&tableStatus, &robot, &cam1, action.getStrategy()->StratStartingPos())){
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
                startMatchHardware();
            }

            // Calibrated and at the starting position: wait for the latch to start the match
            if (sensor.readLatchSensor())
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
                tableStatus.reset();
                tableStatus.startTime = _millis();
                action.Reset();

            }
            bool finished = action.RunFSM();

            if (_millis() > tableStatus.startTime + MATCH_DURATION_MS || finished || sensor.readButtonSensor())
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
                arduino.RGB_Blinking(255, 0, 255); // Purple blinking
            }
            navigation.go();

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
                arduino.RGB_Solid(0, 255, 0);
                manual_clearFunc();
                robot.drive.disable();
                robot.actuators.disableActuators();
                lidar.stopSpin();
                cam1.stop();
                hw_ready = false;
            }

            // Wait for the operator to release the latch, then quit
            if (!sensor.readLatchSensor()){
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

    arduino.RGB_Blinking(255, 0, 0); // Red blinking

#ifndef DISABLE_LIDAR
    if (!lidar.setup("/dev/ttyAMA0", 256000))
    {
        LOG_ERROR("Cannot find the lidar");
        return -1; //TODO handle error
    }
#endif

    // Start the MAT client: its thread retries in the background until the mat
    // is reachable (logs the (re)connection), and is only stopped at exit.
    if (!StartMat())
        LOG_ERROR("MAT not connected at init, retrying in the background");

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

    initState = true;
    manual_init();

    pathfind_setup(tableStatus);

    robot.drive.reset();

    LOG_GREEN_INFO("Init sequence done");
    debugSound::play("start.wav");
    return 0;
}

void GetLidar()
{
    static position_t pos_opponent = OPPONENT_POS_UNKNOWN;
    double IsDataValid = (fabs(robot.drive.velocity.a) <= 45.0) || (position_distance(robot.drive.position, pos_opponent) < 1000);

    if (lidar.getData()){
        convertAngularToAxial(lidar.data, lidar.count, robot.drive.position, 150);
            pathfind_fill_lidar(&lidar, tableStatus);
        // Only update opponent position if the robot is not moving too fast to avoid noise
        if (IsDataValid && position_opponentV2(lidar.data, lidar.count, robot.drive.position, pos_opponent) &&
                (currentState == MainState::Run || currentState == MainState::Manual) &&
                // No match clock yet (startTime == 0): nothing to wait for. Otherwise ignore
                // the first second of the match to avoid false readings at the beginning.
                (tableStatus.startTime == 0 || _millis() - tableStatus.startTime > 1000)){
            tableStatus.pos_opponent.x = pos_opponent.x;
            tableStatus.pos_opponent.y = pos_opponent.y;
            opponentInAction(pos_opponent);

        }
    }
}

void GetMat()
{
    // Le thread de fond de mat_api rafraîchit la table : on ne lit que la
    // dernière donnée, sans bloquer la boucle. Il loggue lui-même la connexion
    // et sa perte, rien à faire ici si la donnée est absente.
    MatTableData matData;
    const bool mastReachable = getMapStatus(matData);
    if (mastReachable)
        tableStatus.updateMapStatus(matData);
    tableStatus.mastStatus = mastReachable;
}

void EndSequence()
{
    LOG_GREEN_INFO("Stopping");

    // Stop the MAT thread (sends GET /stop)
    StopMat();

    // Stop the lidar
    lidar.Stop();
    cam1.stop();

#ifndef EMULATE_I2C
    robot.drive.disable();

    arduino.RGB_Solid(0, 0, 0); // OFF

    for(int i = 0; i < 60; i++){
        if (robot.actuators.homeActuators())
            break;
        delay(100);
    };
    robot.actuators.disableActuators();
#endif // EMULATE_I2C

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

// ------------------------------------------------------
//                        OTHER
// ------------------------------------------------------

// TODO implement this function
/* Detect from position of the opponent the action of the opponent */
void opponentInAction(position_t /*position*/){
}

void switchTeamSide(ColorTeam color){ // TODO move to tableState
    if (color == ColorTeam::None) return;
    if (currentState == MainState::Run) return;
    if (color != tableStatus.colorTeam){
        LOG_INFO("Color switch detected");
        tableStatus.colorTeam = color;

        switch (color)
        {
        case ColorTeam::Blue:
            LOG_INFO("Switching to BLUE");
            arduino.RGB_Blinking(0, 0, 255);
            break;
        case ColorTeam::Yellow:
            LOG_INFO("Switching to YELLOW");
            arduino.RGB_Blinking(255, 56, 0);
            break;
        default:
            break;
        }

        position_t pos = action.getStrategy()->StratStartingPos();
        LOG_INFO("Stratégie '", action.getStrategy()->getNom(),
                 "', départ : x=", pos.x, " y=", pos.y, " a=", pos.a);
    }
}

void switchStrategy(std::string strategy){ // TODO move to tableState
    if (currentState == MainState::Run) return;

    if (strategy != action.getStrategy()->getNom()){
        VirtualStrategy* selectedStrategy = nullptr;
        for (VirtualStrategy* candidate : strategies)
            if (candidate && candidate->getNom() == strategy)
                selectedStrategy = candidate;
        if (!selectedStrategy) return;

        LOG_INFO("Strategy switch detected");
        tableStatus.strategy = strategy;
        action.setStrategy(selectedStrategy);
    }
    else {
        LOG_INFO("Stratégie '", strategy, "' déjà active");
    }
}

void update_team_strat(){
    ColorTeam requestedColor;
    if (api.consumeColorRequest(requestedColor))
        switchTeamSide(requestedColor);

    std::string requestedStrategy;
    if (api.consumeStrategyRequest(requestedStrategy))
        switchStrategy(requestedStrategy);
}
