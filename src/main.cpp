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
#include "utils/utils.h"
#include "utils/logger.hpp"
#include "restAPI/restAPI.hpp"
#include "vision/ArucoCam.hpp"

#define EMULATE_CAM
#ifndef __CROSS_COMPILE_ARM__
    #define DISABLE_LIDAR
    #define TEST_API_ONLY
    #define EMULATE_I2C
#endif


TableState tableStatus;
ActionFSM action;

// Initiation of i2c devices
#ifndef EMULATE_I2C
Asserv asserv(I2C_ASSER_ADDR);
Arduino arduino(I2C_ARDUINO_ADDR);
#else
Asserv asserv(-1);
Arduino arduino(-1);
#endif

Lidar lidar;

#ifndef EMULATE_CAM
ArucoCam arucoCam1(0, "data/brio3.yaml");
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

// Signal Management
bool ctrl_c_pressed = false;
void ctrlc(int)
{
    LOG_INFO("Stop Signal Recieved");
    ctrl_c_pressed = true;
}
bool ctrl_z_pressed = false;
void ctrlz(int signal)
{
    LOG_INFO("Termination Signal Recieved");
    exit(0);
}

int main(int argc, char *argv[])
{
    LOG_DEBUG("Log id is : ",log_asserv()->getLogID());
    if (StartSequence() != 0)
        return -1;

    // Private counter
    unsigned long loopStartTime;
    while (!ctrl_c_pressed)
    {
        loopStartTime = _millis();

        // Get Sensor Data
        {
            int16_t x, y, a;
            asserv.get_coordinates(x, y, a);
            tableStatus.robot.pos = {x, y, a};
            // LOG_GREEN_INFO("Robot pos : { x = ", x," y = ", y, " a = ", a, " }");
            tableStatus.robot.braking_distance = asserv.get_braking_distance();
            asserv.get_current_target(x, y, a);
            tableStatus.robot.target = {x, y, a};
            // LOG_GREEN_INFO("Robot target : { x = ", x," y = ", y, " a = ", a, " }");
            tableStatus.robot.direction_side = (int)asserv.get_direction_side();

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
                if (tableStatus.robot.colorTeam == NONE)
                    arduino.RGB_Blinking(255, 0, 0); // Red Blinking
            }

            // colorTeam_t color = readColorSensorSwitch();
            // switchTeamSide(color);

            if (readLatchSensor() && tableStatus.robot.colorTeam != NONE)
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
                {int16_t x, y, a;
                asserv.get_coordinates(x, y, a);}// for log
                LOG_GREEN_INFO("RUN");
                tableStatus.reset();
                tableStatus.startTime = _millis();
                action.Reset();
                moveColumnsElevator(1);
            }
            bool finished = action.RunFSM();

            if (_millis() > tableStatus.startTime + 100000 || finished || readButtonSensor())
                nextState = FIN;
            break;
        }
        //****************************************************************
        case MANUAL:
        {
            if (initState){
                LOG_GREEN_INFO("MANUAL");
                arduino.RGB_Blinking(255, 0, 255); // Purple blinking
            }

            // Execute the function as long as it returns false
            if (manual_currentFunc != NULL && manual_currentFunc != nullptr){
                if (manual_currentFunc()){
                    manual_currentFunc = NULL;
                }
            }
            if (!manual_ctrl)
                nextState = FIN;
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
                asserv.stop();
                // Clear manual_func
                manual_currentFunc = NULL;
                lidar.stopSpin();
            }

            if (!readLatchSensor()){
                enableActuators();
                ctrl_c_pressed = true; // nextState = INIT;
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

        // asserv.logAsserv();
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
        return -1;
    }
#endif

    // Start the api server in a separate thread
    api_server_thread = std::thread([&]()
                                    { StartAPIServer(); });

#ifdef TEST_API_ONLY
    TestAPIServer();
    sleep(1);
    LOG_DEBUG("Starting main debug loop");
    int i = 0;
    while(!ctrl_c_pressed){
        sleep(0.1);
        // randomly change the position of highway obstacles
        position_t t_pos = {rand() % 1500 - 750, rand() % 2200 - 1100, 0};
        navigationGoTo(t_pos, Direction::FORWARD, Rotation::SHORTEST, Rotation::SHORTEST);
    }
    StopAPIServer();
    api_server_thread.join();
    return -1;
#endif

    // TODO Remove (For testing)
    // TestRevolver();

    currentState = INIT;
    nextState = INIT;
    initState = true;
    manual_ctrl = false;
    manual_currentFunc = NULL;

    asserv.set_coordinates(0,0,0);

    LOG_GREEN_INFO("Init sequence done");
    return 0;
}

void GetLidar()
{
    // Implements a low passfilter
    // Simple exponential moving average (EMA)
    
    // Smoothing factor (0 < alpha < 1)
    const float alpha = 0.3f; // Adjust this value for more or less smoothing on the opponent robot posititon

    static position_t pos_opponent_filtered = {0, 0, 0};
    static bool first_reading = true;

    if (lidar.getData())
    {
        position_t position = tableStatus.robot.pos;
        convertAngularToAxial(lidar.data, lidar.count, position, 200);
        
        if (currentState == RUN || currentState == MANUAL)
            navigationOpponentDetection();
        
        position_t pos_opponent;
        if (position_opponentV2(lidar.data, lidar.count, position, &pos_opponent)){
            // If it's the first reading, initialize the filtered position
            if (first_reading)
            {
                pos_opponent_filtered.x = pos_opponent.x;
                pos_opponent_filtered.y = pos_opponent.y;
                first_reading = false;
            }
            else
            {
                // Apply the low-pass filter
                pos_opponent_filtered.x = alpha * pos_opponent.x + (1 - alpha) * pos_opponent_filtered.x;
                pos_opponent_filtered.y = alpha * pos_opponent.y + (1 - alpha) * pos_opponent_filtered.y;
            }

            // Save the filtered position to tableStatus
            tableStatus.pos_opponent.x = pos_opponent_filtered.x;
            tableStatus.pos_opponent.y = pos_opponent_filtered.y;
            if ((currentState == RUN || currentState == MANUAL) && (_millis() > tableStatus.startTime + 1000))
                opponentInAction(pos_opponent_filtered);            
        }
    }
}

void EndSequence()
{
    LOG_GREEN_INFO("Stopping");
    
    // Stop the lidar
    lidar.Stop();

#ifndef EMULATE_I2C
    asserv.stop();
    asserv.set_motor_state(false);
    asserv.set_brake_state(false);

    arduino.RGB_Solid(0, 0, 0); // OFF

    for(int i = 0; i < 60; i++){
        if (homeActuators() & moveColumnsElevator(0) & movePlatformElevator(-1))
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