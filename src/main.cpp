#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <thread>
#include <unistd.h>  // for usleep
#include <math.h>

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
    // Aruco tags are at pos (+/-400, +/-900)
    position_t pos_otos, pos_cam;
    int current_test = 0;
    nav_return_t ret;
    
    static bool state = false;

    switch (current_test){
    case 0:{
        // Camera calibration test
        // Spin the robot on the spot and calculate the dx dy da of the camera,
        static bool has_prev_measure = false;
        static position_t prev_pos_cam = {0.0, 0.0, 0.0};
        static position_t prev_pos_otos = {0.0, 0.0, 0.0};

        position_t pos1 = {0.0, 900.0, 0.0};
        if (state)
            pos1.a = 180.0;
        ret = navigationGoTo(pos1, false, true, true, &pos_cam, &pos_otos);
        if (ret == NAV_DONE){
            //LOG_GREEN_INFO("", pos_otos.x, ", ", pos_otos.y, ", ", pos_otos.a);
            //LOG_GREEN_INFO("", pos_cam.x, ", ", pos_cam.y, ", ", pos_cam.a);
            constexpr double PI = 3.14159265358979323846;

            if (!has_prev_measure){
                prev_pos_cam = pos_cam;
                prev_pos_otos = pos_otos;
                has_prev_measure = true;
                LOG_GREEN_INFO("Cam calibration: first pose recorded");
            } else {
                position_t delta = {0.0, 0.0, 0.0};

                // Solve for camera offset d in robot frame from:
                // (c2 - c1) - (o2 - o1) = (R(a2) - R(a1)) * d
                const double a1 = prev_pos_otos.a * PI / 180.0;
                const double a2 = pos_otos.a * PI / 180.0;

                const double c1 = cos(a1);
                const double s1 = sin(a1);
                const double c2 = cos(a2);
                const double s2 = sin(a2);

                const double ax = c2 - c1;
                const double bx = s1 - s2;
                const double det = ax * ax + bx * bx;

                if (det > 1e-9){
                    const double d_cam_x = pos_cam.x - prev_pos_cam.x;
                    const double d_cam_y = pos_cam.y - prev_pos_cam.y;
                    const double d_robot_x = pos_otos.x - prev_pos_otos.x;
                    const double d_robot_y = pos_otos.y - prev_pos_otos.y;

                    const double dcx = d_cam_x - d_robot_x;
                    const double dcy = d_cam_y - d_robot_y;

                    delta.x = (ax * dcx - bx * dcy) / det;
                    delta.y = (bx * dcx + ax * dcy) / det;

                    const double da1 = normalize_angle(prev_pos_cam.a - prev_pos_otos.a);
                    const double da2 = normalize_angle(pos_cam.a - pos_otos.a);
                    delta.a = normalize_angle((da1 + da2) * 0.5);

                    LOG_GREEN_INFO("Cam offset (rotation calib) dx=", delta.x, " dy=", delta.y, " da=", delta.a);
                } else {
                    LOG_WARNING("Cam calibration skipped: rotation too small between samples");
                }

                prev_pos_cam = pos_cam;
                prev_pos_otos = pos_otos;
            }

            state = !state;
        }
    } break;
    case 1:{
        // Angle compensation test
        // New Scalar = Current Scalar * (Actual Angle / Reported Angle)
        static bool has_prev_measure = false;
        static position_t prev_pos = {0.0, 0.0, 0.0};

        position_t pos1 = {0.0, 900.0, 0.0};
        if (state)
            pos1.a = 180.0;
        ret = navigationGoTo(pos1, false, true, false, &pos_cam, &pos_otos);

        if (ret == NAV_DONE){
            if (!has_prev_measure){
                prev_pos = pos_cam;
                has_prev_measure = true;
                LOG_GREEN_INFO("OTOS angle calibration: first pose recorded");
            } else {
                const double da_cam = normalize_angle(pos_cam.a - prev_pos.a);
                const double da_otos = normalize_angle(pos_otos.a - prev_pos.a);

                if (fabs(da_otos) > 1e-3){
                    const double scalar_angle = da_cam / da_otos;
                    LOG_GREEN_INFO("OTOS scalar angle=", scalar_angle,
                                   " (actual=", da_cam, " reported=", da_otos, ")");
                } else {
                    LOG_WARNING("OTOS angle calibration skipped: reported angle too small");
                }

                prev_pos = pos_cam;
            }

            state = !state;
        }
    } break;
    case 2:{
        // Position compensation test, also compensate for otos angle
        // New Scalar = Current Scalar * (Actual Distance / Reported Distance)
        static bool has_prev_measure = false;
        static position_t prev_pos_cam = {0.0, 0.0, 0.0};
        static position_t prev_pos_otos = {0.0, 0.0, 0.0};

        position_t pos1 = {0.0, 900.0, 0.0};
        if (state)
            pos1.y *= -1.0;
        ret = navigationGoTo(pos1, false, true, false, &pos_cam, &pos_otos);

        if (ret == NAV_DONE){
            if (!has_prev_measure){
                prev_pos_cam = pos_cam;
                prev_pos_otos = pos_otos;
                has_prev_measure = true;
                LOG_GREEN_INFO("OTOS scalar calibration: first pose recorded");
            } else {
                const double d_cam_x = pos_cam.x - prev_pos_cam.x;
                const double d_cam_y = pos_cam.y - prev_pos_cam.y;
                const double d_otos_x = pos_otos.x - prev_pos_otos.x;
                const double d_otos_y = pos_otos.y - prev_pos_otos.y;

                const double dist_cam = sqrt(d_cam_x * d_cam_x + d_cam_y * d_cam_y);
                const double dist_otos = sqrt(d_otos_x * d_otos_x + d_otos_y * d_otos_y);

                if (dist_otos > 1e-6){
                    const double scalar_dist = dist_cam / dist_otos;
                    LOG_GREEN_INFO("OTOS scalar distance=", scalar_dist,
                                   " (actual=", dist_cam, " reported=", dist_otos, ")");
                } else {
                    LOG_WARNING("OTOS scalar calibration skipped: reported displacement too small");
                }

                prev_pos_cam = pos_cam;
                prev_pos_otos = pos_otos;
            }

            state = !state;
        }
    } break;
    }
}