#include "main.hpp"

#include <stdlib.h>
#include <signal.h>
#include <cmath>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>  // for usleep

#include "utils/logger.hpp"

//lidar related
#include "lidar/Lidar.hpp"
#include "lidar/lidarAnalize.h"

//navigation related
#include "navigation/navigation.h"
#include "navigation/pathfind.h"
#include "actions/calibration.h"

#include "utils/utils.h"
#include "restAPI/restAPI.hpp"
#include "restAPI/manual_mode.h"
#include "vision/Cam.hpp"
#include "mat/mat.hpp"

//brain include
#include "actions/MainActionFSM.hpp"
#include "actions/Strategy/ExempleStrat.hpp"

//Arduino
#include "i2c/Arduino.hpp"
#include "actions/SensorControl.hpp"
#include "actions/ActuatorsControl.hpp"

#ifndef __CROSS_COMPILE_ARM__
    #define DISABLE_LIDAR
    #define TEST_API_ONLY
    #define EMULATE_I2C
#endif

// Hardware init
DriveControl drive;
Arduino arduino;
Lidar lidar= Lidar(&arduino);

// control of hardware
SensorControl sensor(&arduino);
ActuatorsControl actuators(&arduino, &drive);

// status of table
TableState tableStatus(&drive);

// Brain Init
VirtualStrategy* currentStrategy = new ExempleStrat(&drive, &tableStatus);

// Strategies selectable through the REST API
std::vector<VirtualStrategy*> strategies = { currentStrategy };

ActionFSM action(currentStrategy, &drive, &tableStatus);

#ifndef EMULATE_CAM
Cam cam1 = Cam(0, "data/OV9281_1280_800.yaml", "data/table.png");
#else
Cam cam1(-1, "", "");
#endif

// Navigation
Navigation navigation(&drive, &tableStatus, &cam1);

main_State_t currentState;
main_State_t nextState;
bool initState;

std::thread api_server_thread;

// REST API
RestAPI api(&currentState, &nextState, &drive, &tableStatus, &arduino, &lidar, &cam1, &navigation, &strategies);

// Prototypes
int StartSequence();
void GetLidar();
void EndSequence();
void tests();
bool tryConnectMat();

// Signal Management
bool exit_requested = false;
// Set by --unit-test: the TEST state runs the bench checks once and exits.
bool unitTestMode = false;
void ctrlc(int)
{
    LOG_INFO("Stop Signal Received");
    exit_requested = true;
}

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--unit-test")
            unitTestMode = true;
    }

    LOG_DEBUG("LOG ID : ", log_main_get_id());
    if (StartSequence() != 0)
        return -1;

    while (!exit_requested)
    {
        const unsigned long loopStartTime = _millis();

        // Get Sensor Data
        {
            drive.update();
            // The odometry prior helps the feature localiser; the marker
            // localiser ignores it.
            cam1.setPrior(drive.position);

            if (currentState != INIT && currentState != FIN)
            {
#ifndef DISABLE_LIDAR
                GetLidar();
#endif
                if (tableStatus.mastStatus) { // TODO make request async to reduce delay issue in case of deconnection
                    if (getMapStatus()) { // Getting data from mast
                        tableStatus.updateMapStatus();
                    } else {
                        LOG_ERROR("Failed to get map status from mast");
                        tableStatus.mastStatus = false; // Don't try to get mast information for the rest of the match
                    }
                }
            }
        }

        // State machine
        switch (currentState)
        {
        //****************************************************************
        case INIT:
        {
            static bool matStarted = false;
            if (initState)
            {
                LOG_GREEN_INFO("INIT");
                actuators.disableActuators();
                tableStatus.reset();
                arduino.RGB_Rainbow();
            }
            if (!matStarted)
                matStarted = tryConnectMat();

            update_team_strat();

            if (sensor.readButtonSensor() && !sensor.readLatchSensor() && tableStatus.colorTeam != NONE && currentStrategy != nullptr)
                nextState = WAITSTART;
            break;
        }
        //****************************************************************
        case WAITSTART:
        {
            if (initState){
                LOG_GREEN_INFO("WAITSTART");
                actuators.enableActuators();
                actuators.homeActuators();
                lidar.startSpin();
                cam1.start();

                if (tableStatus.colorTeam == NONE)
                    arduino.RGB_Blinking(255, 0, 0); // Red Blinking
                tableStatus.calibrationAge = -1;
            }

            if (tableStatus.calibrationAge == -1){
                navigation.go();
            } else{
                //nextState = CALIBRATION;
            }

            if (sensor.readLatchSensor() && tableStatus.colorTeam != NONE)
                nextState = RUN;
            if (manual_ctrl)
                nextState = MANUAL;
            break;
        }
        //****************************************************************
        case CALIBRATION:
        {
            if (initState){
                LOG_GREEN_INFO("CALIBRATION");
            }
            tableStatus.pos_opponent = OPPONENT_POS_UNKNOWN;

            tableStatus.startTime = _millis();
            static bool has_calib = false;
            if (!has_calib){
                if (calibrate_otos(&tableStatus, &drive, action.getStrategy()->StratStartingPos())){
                    LOG_GREEN_INFO("Calibration successful");
                    has_calib = true;
                }
            }

            if (sensor.readLatchSensor() && tableStatus.colorTeam != NONE)
                nextState = RUN;
            if (manual_ctrl)
                nextState = MANUAL;
            break;
        }
        //****************************************************************
        case RUN:
        {
            if (initState){
                LOG_GREEN_INFO("RUN");
                tableStatus.reset();
                tableStatus.startTime = _millis();
                action.Reset();

            }
            bool finished = action.RunFSM();

            if (_millis() > tableStatus.startTime + MATCH_DURATION_MS || finished || sensor.readButtonSensor())
                nextState = FIN;
            break;
        }
        //****************************************************************
        case TEST:
        {
            if (initState){
                LOG_GREEN_INFO("TEST");
            }

            // The bench has no MAT; keep its connection errors out of the checks.
            if (!unitTestMode && !tableStatus.mastStatus)
                tryConnectMat();

            tests();
            break;
        }
        //****************************************************************
        case MANUAL:
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
        case FIN:
        {
            if (initState){
                LOG_GREEN_INFO("FIN");
                arduino.RGB_Solid(0, 255, 0);
                manual_clearFunc();
                drive.disable();
                actuators.disableActuators();
                lidar.stopSpin();
                StopMat();
            }

            if (!sensor.readLatchSensor()){
                actuators.enableActuators();
                exit_requested = true;
            }
            break;
        }
        //****************************************************************
        default:
            nextState = INIT;
            break;
        }

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

    // Start the api server in a separate thread
    api_server_thread = std::thread([&]()
                                    { api.start(); });

#ifdef TEST_API_ONLY
    LOG_GREEN_INFO("Running in API test mode only");
    currentState = TEST;
    nextState = TEST;
#else
    if (unitTestMode) {
        LOG_GREEN_INFO("Running hardware checks (--unit-test)");
        currentState = TEST;
        nextState = TEST;
    } else {
        currentState = INIT;
        nextState = INIT;
    }
#endif // TEST_API_ONLY

    initState = true;
    manual_init();

    pathfind_setup();

    drive.reset();

    LOG_GREEN_INFO("Init sequence done");
    return 0;
}

void GetLidar()
{
    static position_t pos_opponent = OPPONENT_POS_UNKNOWN;
    double IsDataValid = (fabs(drive.velocity.a) <= 45.0) || (position_distance(drive.position, pos_opponent) < 1000);

    if (lidar.getData()){
        convertAngularToAxial(lidar.data, lidar.count, drive.position, 150);
        pathfind_fill_lidar(&lidar);
        // Only update opponent position if the robot is not moving too fast to avoid noise
        if (IsDataValid && position_opponentV2(lidar.data, lidar.count, drive.position, pos_opponent) &&
                (currentState == RUN || currentState == MANUAL) &&
                (_millis() - tableStatus.startTime > 1000)){ // Only update opponent position after 1 second from the start to avoid false readings at the beginning
            tableStatus.pos_opponent.x = pos_opponent.x;
            tableStatus.pos_opponent.y = pos_opponent.y;
            opponentInAction(pos_opponent);

        }
    }
}

void EndSequence()
{
    LOG_GREEN_INFO("Stopping");

    // Stop the lidar
    lidar.Stop();
    cam1.stop();

#ifndef EMULATE_I2C
    drive.disable();

    arduino.RGB_Solid(0, 0, 0); // OFF

    for(int i = 0; i < 60; i++){
        if (actuators.homeActuators())
            break;
        delay(100);
    };
    actuators.disableActuators();
#endif // EMULATE_I2C

    // Stop the API server
    api.stop();
    api_server_thread.join();

    LOG_GREEN_INFO("Stopped");
}


// ------------------------------------------------------
//                HARDWARE CHECKS (HIL)
// ------------------------------------------------------
// Read-only bench checks: the camera is started and its controls are written,
// the lidar motor is spun up to produce a scan, and both boards are queried.
// No actuator motion is commanded.
namespace {

constexpr int FRAME_TIMEOUT_MS = 5000;
constexpr int LIDAR_TIMEOUT_MS = 5000;
constexpr int LIDAR_MIN_POINTS = 50;
constexpr double LIDAR_MAX_RANGE_MM = 12000.0;

int checkFailures = 0;

bool check(const char* name, bool ok)
{
    if (ok) {
        LOG_GREEN_INFO("HIL PASS  ", name);
    } else {
        LOG_ERROR("HIL FAIL  ", name);
        checkFailures++;
    }
    return ok;
}

void skip(const char* name)
{
    LOG_WARNING("HIL SKIP  ", name);
}

bool sameFloat(float a, float b)
{
    return std::fabs(a - b) < 1e-5f;
}

bool waitForFrame()
{
    std::vector<uchar> jpeg;
    const unsigned long start = _millis();
    while (_millis() - start < FRAME_TIMEOUT_MS) {
        if (cam1.getRawPreview(jpeg) && !jpeg.empty())
            return true;
        usleep(50 * 1000);
    }
    return false;
}

void testCamera()
{
    cam1.start();
    if (!check("camera: a frame arrives", waitForFrame()))
        return;

    // Write the three hardware controls, then check that the values read back
    // and that frames still arrive. The previous values are restored so the
    // bench is left as it was found.
    const float ev = cam1.getExposureValue();
    const float contrast = cam1.getContrast();
    const float brightness = cam1.getBrightness();

    cam1.setExposureValue(ev + 1.0f);
    cam1.setContrast(contrast + 0.2f);
    cam1.setBrightness(brightness + 0.1f);

    check("camera: exposure value round-trip", sameFloat(cam1.getExposureValue(), ev + 1.0f));
    check("camera: contrast round-trip", sameFloat(cam1.getContrast(), contrast + 0.2f));
    check("camera: brightness round-trip", sameFloat(cam1.getBrightness(), brightness + 0.1f));
    check("camera: frames still arrive after a control change", waitForFrame());

    cam1.setExposureValue(ev);
    cam1.setContrast(contrast);
    cam1.setBrightness(brightness);
}

void testLidar()
{
#ifdef DISABLE_LIDAR
    skip("lidar: not built on this target (DISABLE_LIDAR)");
#else
    lidar.startSpin();

    bool gotScan = false;
    const unsigned long start = _millis();
    while (_millis() - start < LIDAR_TIMEOUT_MS) {
        if (lidar.getData() && lidar.count > 0) {
            gotScan = true;
            break;
        }
        usleep(50 * 1000);
    }
    lidar.stopSpin();

    if (!check("lidar: a scan arrives", gotScan))
        return;

    check("lidar: point count is not empty", lidar.count > 0 && lidar.count <= SIZEDATALIDAR);

    // Only the ranges the sensor can actually measure are meaningful; a bench
    // with little in view still yields a few of them.
    int plausible = 0;
    double shortest = LIDAR_MAX_RANGE_MM;
    double longest = 0.0;
    for (int i = 0; i < lidar.count; i++) {
        const double dist = lidar.data[i].dist;
        if (dist > 0.0 && dist <= LIDAR_MAX_RANGE_MM) {
            plausible++;
            if (dist < shortest) shortest = dist;
            if (dist > longest) longest = dist;
        }
    }
    LOG_INFO("HIL lidar: ", lidar.count, " points, ", plausible,
             " with a plausible range (", shortest, " .. ", longest, " mm)");

    check("lidar: enough plausible ranges", plausible >= LIDAR_MIN_POINTS);
    check("lidar: ranges are not all identical", longest > shortest);
#endif
}

void testI2c()
{
#ifdef EMULATE_I2C
    skip("i2c: boards are emulated on this target (EMULATE_I2C)");
#else
    // Arduino board: read-only queries, nothing is commanded to move.
    bool sensor = false;
    check("i2c: arduino answers (sensor read)", arduino.readSensor(BUTTON_SENSOR_NUM, sensor));

    int32_t stepper = 0;
    check("i2c: arduino answers (stepper read)", arduino.getStepper(stepper, STEPPER_NUM_2));

    int servo = 0;
    check("i2c: arduino answers (servo read)", arduino.getServo(SERVO_CLAW_CLOSE_1, servo));

    // Drive board: update() reads the motion and the target over I2C.
    drive.update();
    LOG_INFO("HIL drive: position (", drive.position.x, ", ", drive.position.y, ", ",
             drive.position.a, ")");
    check("i2c: drive board answers (motion read)",
          std::isfinite(drive.position.x) && std::isfinite(drive.position.y) &&
              std::isfinite(drive.position.a));
#endif
}

} // namespace

void tests()
{
    // TEST is also the host API-test state, which runs no hardware checks.
    if (!unitTestMode)
        return;

    static bool alreadyRun = false;
    if (alreadyRun)
        return;
    alreadyRun = true;

    LOG_GREEN_INFO("===== HIL hardware checks =====");
    testCamera();
    testLidar();
    testI2c();
    LOG_GREEN_INFO("===== HIL hardware checks done: ", checkFailures, " failure(s) =====");

    // EndSequence() stops the camera and the lidar for us.
    exit_requested = true;
}

// ------------------------------------------------------
//                        OTHER
// ------------------------------------------------------

// Tries to connect to the MAT. Returns true once the request was accepted,
// and sets tableStatus.mastStatus when the link is actually up.
bool tryConnectMat()
{
    bool connectionOk = false;
    const bool started = StartMat(connectionOk);
    if (connectionOk)
    {
        tableStatus.mastStatus = true;
        LOG_GREEN_INFO("MAT is ready");
    }
    return started;
}

bool returnToHome(){

    static position_t homePos;
    nav_return_t res = navigation.goTo(homePos, true);
    if (res == NAV_ERROR){
        LOG_ERROR("RETURN_TO_HOME: Navigation error");
        homePos.y += (tableStatus.colorTeam == BLUE) ? 50 : -50; // recule un peu et retente
    }
    if (res == NAV_DONE){
        LOG_GREEN_INFO("RETURN_TO_HOME: Done");
        return true;
    }
    return false;
}

// TODO implement this function
/* Detect from position of adversary the action of the adversary */
void opponentInAction(position_t /*position*/){
}

void switchTeamSide(colorTeam_t color){ // TODO move to tableState
    if (color == NONE) return;
    if (currentState == RUN) return;
    if (color != tableStatus.colorTeam){
        LOG_INFO("Color switch detected");
        tableStatus.colorTeam = color;

        switch (color)
        {
        case BLUE:
            LOG_INFO("Switching to BLUE");
            arduino.RGB_Blinking(0, 0, 255);
            break;
        case YELLOW:
            LOG_INFO("Switching to YELLOW");
            arduino.RGB_Blinking(255, 56, 0);
            break;
        default:
            break;
        }

        position_t pos = action.getStrategy()->StratStartingPos();
        LOG_INFO("Stratégie '", action.getStrategy()->getNom(),
                 "', départ : x=", pos.x, " y=", pos.y, " a=", pos.a);
        drive.setCoordinates(pos);
        navigation.goTo(pos, true, true); // Go to starting pos with A* and slow mode to avoid collisions during the switch
    }
}

void switchStrategy(std::string strategy){ // TODO move to tableState
    if (currentState == RUN) return;

    if (strategy != action.getStrategy()->getNom()){
        VirtualStrategy* selectedStrategy = nullptr;
        for (VirtualStrategy* candidate : strategies)
            if (candidate && candidate->getNom() == strategy)
                selectedStrategy = candidate;
        if (!selectedStrategy) return;

        LOG_INFO("Strategy switch detected");
        tableStatus.strategy = strategy;
        action.setStrategy(selectedStrategy);
        position_t pos = action.getStrategy()->StratStartingPos();
        drive.setCoordinates(pos);
    }
    else {
        LOG_INFO("Stratégie '", strategy, "' déjà active");
    }
}

void update_team_strat(){
    colorTeam_t requestedColor;
    if (api.consumeColorRequest(requestedColor))
        switchTeamSide(requestedColor);

    std::string requestedStrategy;
    if (api.consumeStrategyRequest(requestedStrategy))
        switchStrategy(requestedStrategy);
}
