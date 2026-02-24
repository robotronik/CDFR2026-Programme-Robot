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

#ifndef __CROSS_COMPILE_ARM__
    #define DISABLE_LIDAR
    #define TEST_API_ONLY
    #define EMULATE_CAM
    #define EMULATE_I2C
#endif


TableState tableStatus;
ActionFSM action;

DriveControl drive;
Arduino arduino;

Lidar lidar;

#ifndef EMULATE_CAM
ArucoCam arucoCam1(0, "data/OV9281_1280_800.yaml");
#else
ArucoCam arucoCam1(-1, "");
#endif

main_State_t currentState;
main_State_t nextState;
bool initState;
bool manual_ctrl;
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
void ctrlz(int signal)
{
    LOG_INFO("Termination Signal Recieved");
    exit(0);
}

int main(int argc, char *argv[])
{
    LOG_DEBUG("LOG ID : ",log_asserv()->getLogID());
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

            if (readButtonSensor() & !readLatchSensor())
                nextState = WAITSTART;
            break;
        }
        //****************************************************************
        case WAITSTART:
        {
            if (initState){
                LOG_GREEN_INFO("WAITSTART");  
                enableActuators();
                arduino.setStepper(0, 1);
                arduino.setStepper(0, 2);
                arduino.setStepper(0, 3);
                arduino.setStepper(0, 4);
                homeActuators();
                lidar.startSpin();
                arduino.moveMotorDC(130, false);

                if (tableStatus.colorTeam == NONE)
                    arduino.RGB_Blinking(255, 0, 0); // Red Blinking
            }

            // colorTeam_t color = readColorSensorSwitch();
            // switchTeamSide(color);

            if (readLimitSwitchTop()){
                arduino.stopMotorDC();
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
                log_asserv()->setLogStatus(true);
                LOG_GREEN_INFO("RUN");
                tableStatus.reset();
                tableStatus.startTime = _millis();
                action.Reset();
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
                log_asserv()->setLogStatus(false);
                arduino.RGB_Solid(0, 255, 0);
                disableActuators();
                // Clear command buffer
                drive.disable();
                // Clear manual_func
                manual_currentFunc = NULL;
                lidar.stopSpin();
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

#ifdef TEST_API_ONLY
    LOG_GREEN_INFO("Running in API test mode only");
    currentState = TEST;
    nextState = TEST;
#else
    currentState = INIT;
    nextState = INIT;
#endif // TEST_API_ONLY

    initState = true;
    manual_ctrl = false;
    manual_currentFunc = NULL;

    pathfindInit();

    // TODO REMOVE
    pathfind_place_obstacle_rect_with_inflation(400, 0, 100, 100, 200);


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

            if ((currentState == RUN || currentState == MANUAL) && (_millis() > tableStatus.startTime + 1000))
                opponentInAction(pos_opponent);            
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
    TestAPIServer();
    pathfind_setup();
    
    while(!exit_requested){
        usleep(500000); 

        //position depart et arrivé aléatoire
        int start_ix = 0, start_iy = 0, goal_ix = 0, goal_iy = 0;
        position_t target, start, goal;
        target.x = rand() % (600) - 600 / 2;
        target.y = rand() % (600) - 600 / 2;

        start.x = rand() % (1400) - 1400 / 2;
        start.y = rand() % (1400) - 1400 / 2;
        start.a = rand() % 360;
        drive.setCoordinates(start);
        
        navigationGoTo(target, true, true);
    }
    StopAPIServer();
    api_server_thread.join();

    LOG_GREEN_INFO("Tests finished");
    nextState = FIN;    
}