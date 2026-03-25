#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <thread>
#include <unistd.h>  // for usleep

#include "main.hpp"
#include "actions/action.hpp"
#include "actions/functions.h"
#include "lidar/lidarAnalize.h"
#include "navigation/navigation.h"
#include "navigation/pathfind.h"
#include "utils/utils.h"
#include "utils/logger.hpp"
#include "restAPI/restAPI.hpp"

#define TEST_MODE
#ifndef __CROSS_COMPILE_ARM__
    #define DISABLE_LIDAR
    #define EMULATE_I2C
#endif


TableState tableStatus;
ActionFSM action;

DriveControl drive;
Arduino arduino;

Lidar lidar;

#ifndef EMULATE_CAM
ArucoCam arucoCam1 = ArucoCam(0, "data/OV9281_1280_800.yaml");
#else
ArucoCam arucoCam1(-1, "");
#endif

main_State_t currentState;
main_State_t nextState;
bool initState;
bool manual_ctrl;
bool motorUpFirst = true;
bool (*manual_currentFunc)(); //Pointer to a function to execute of type bool func(void)

std::thread api_server_thread;

// Prototypes
int StartSequence();
void GetLidar();
void EndSequence();
void tests();

// Signal Management
bool exit_requested = false;
void ctrlc(int)
{
    LOG_INFO("Stop Signal Recieved");
    exit_requested = true;
}
void ctrlz(int)
{
    LOG_INFO("Termination Signal Recieved");
    exit(0);
}

int main(int argc, char *argv[])
{
    LOG_DEBUG("LOG ID : ", log_main_get_id());
    if (StartSequence() != 0)
        return -1;

    // Private counter
    unsigned long loopStartTime;
    while (!exit_requested)
    {
        loopStartTime = _millis();

        // Get Sensor Data
        {
            drive.update();
            //LOG_INFO("x: ", drive.position.x, " y: ", drive.position.y, " a: ", drive.position.a);

            if (currentState != INIT && currentState != FIN)
            {
#ifndef DISABLE_LIDAR
                GetLidar();
#endif
            }
        }

        // State machine
        switch (currentState)
        {
        //****************************************************************
        case INIT:
        {
            if (initState)
            {
                LOG_GREEN_INFO("INIT");
                disableActuators();
                tableStatus.reset();
                arduino.RGB_Rainbow();
            }

            if (readButtonSensor() && !readLatchSensor())
                nextState = WAITSTART;
            break;
        }
        //****************************************************************
        case WAITSTART:
        {   
            static nav_return_t nav_ret;
            if (initState){
                LOG_GREEN_INFO("WAITSTART");  
                enableActuators();
                arduino.setStepper(0, 1);
                arduino.setStepper(0, 2);
                arduino.setStepper(0, 3);
                arduino.setStepper(0, 4);
                homeActuators();
                lidar.startSpin();
                arucoCam1.start();
                arduino.moveMotorDC(80, false);

                if (tableStatus.colorTeam == NONE)
                    arduino.RGB_Blinking(255, 0, 0); // Red Blinking
                nav_ret = NAV_IN_PROCESS;
                tableStatus.calibrationAge = -1;
            }

            #ifdef TEST_MODE
                LOG_GREEN_INFO("Running in Test mode");
                nextState = TEST;
            #endif
            // colorTeam_t color = readColorSensorSwitch();
            // switchTeamSide(color);

            if (readLimitSwitchTop() && motorUpFirst){ // Why TF is there an "and" 
                arduino.moveMotorDC(20,false);
                motorUpFirst = false;
            }
            if (tableStatus.calibrationAge ==-1){
                navigationGo(); // Calibrate the robot using the camera while waiting for the start signal
            } else if (nav_ret == NAV_IN_PROCESS){
                nav_ret = navigationGo();
            }
            if(nav_ret == NAV_ERROR){
                LOG_ERROR("Error while calibrating in WAITSTART");
            }

            if (readLatchSensor() && tableStatus.colorTeam != NONE)
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
                arduino.keepMotorDCup();

            }
            bool finished = action.RunFSM();

            if (_millis() > tableStatus.startTime + 100000 || finished || readButtonSensor())
                nextState = FIN;
            break;
        }
        //****************************************************************
        case TEST:
        {
            if (initState){
                LOG_GREEN_INFO("TEST");
            }
            // Run tests
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
            navigationGo();

            // Execute the function as long as it returns false
            if (manual_currentFunc != NULL && manual_currentFunc != nullptr){
                if (manual_currentFunc()){
                    manual_currentFunc = NULL;
                }
            }
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
                disableActuators();
                // Clear command buffer
                drive.disable();
                // Clear manual_func
                manual_currentFunc = NULL;
                lidar.stopSpin();
                arduino.stopMotorDC();
            }

            if (!readLatchSensor()){
                enableActuators();
                exit_requested = true;
            }
            break;
        }
        //****************************************************************
        default:
            LOG_GREEN_INFO("default");
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
    // signal(SIGTSTP, ctrlz);

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
                                    { StartAPIServer(); });

    currentState = INIT;
    nextState = INIT;

    initState = true;
    manual_ctrl = false;
    manual_currentFunc = NULL;

    pathfind_setup();

    drive.reset();

    LOG_GREEN_INFO("Init sequence done");
    return 0;
}

void GetLidar()
{
    static position_t prev_pos;
    static position_t prev_vel;
    static long prev_time_ms = 0;

    if (lidar.getData())
    {
        double time_s = double(_millis() - prev_time_ms) / 1000.0; 
        //convertAngularToAxial(lidar.data, lidar.count, position, 200);
        convertAngularToAxialCompensated(lidar.data, lidar.count, prev_pos, prev_vel, time_s, 200);
        pathfind_fill_lidar();
        
        if (currentState == RUN || currentState == MANUAL)
            navigationOpponentDetection();
        
        position_t pos_opponent;
        if (position_opponentV2(lidar.data, lidar.count, drive.position, pos_opponent)){
            // If it's the first reading, initialize the filtered position
            // Apply the low-pass filter
            tableStatus.pos_opponent.x = pos_opponent.x;
            tableStatus.pos_opponent.y = pos_opponent.y;

            if ((currentState == RUN || currentState == MANUAL) && (_millis() > tableStatus.startTime + 1000) && (fabs(drive.velocity.a) <= 45)){ // Only update opponent position if the robot is not moving too fast to avoid noise, and after 1 second from the start to avoid false readings at the beginning
                opponentInAction(pos_opponent);            
            }
        }

        prev_pos = drive.position;
        prev_vel = drive.velocity;
        prev_time_ms = _millis();
    }
}

void EndSequence()
{
    LOG_GREEN_INFO("Stopping");
    
    // Stop the lidar
    lidar.Stop();
    arucoCam1.stop();

#ifndef EMULATE_I2C
    drive.disable();

    arduino.RGB_Solid(0, 0, 0); // OFF

    for(int i = 0; i < 60; i++){
        if (homeActuators())
            break;
        delay(100);
    };
    disableActuators();
#endif // EMULATE_I2C

    // Stop the API server
    StopAPIServer();
    api_server_thread.join();

    LOG_GREEN_INFO("Stopped");
}


void tests()
{
    static bool state = false;
    position_t pos1 = {400.0, 0.0, -90.0};
    pos1.y = state ? 900 : -900.0;
    pos1.y += 300.0;
    nav_return_t ret = navigationGoTo(pos1, false, true);
    if (ret == NAV_DONE)
        state = !state;

    //New Scalar = Current Scalar * (Actual Distance / Reported Distance)
}