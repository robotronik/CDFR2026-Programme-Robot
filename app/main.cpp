#pragma once
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>  // for usleep

#include "drive_interface.h"
#include "defs/structs.hpp"

#include "utils/logger.hpp" // logger 

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
#include "vision/ArucoCam.hpp"
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
VirtualStrategy* currentStrategy = new ExempleStrat(&drive, &tableStatus); //TODO create logic for strategy change

// Strategies selectable through the REST API
std::vector<VirtualStrategy*> strategies = { currentStrategy };

ActionFSM action(currentStrategy, &drive, &tableStatus);

#ifndef EMULATE_CAM
ArucoCam arucoCam1 = ArucoCam(0, "data/OV9281_1280_800.yaml");
#else
ArucoCam arucoCam1(-1, "");
#endif

// Navigation
Navigation navigation(&drive, &tableStatus, &arucoCam1);

main_State_t currentState;
main_State_t nextState;
bool initState;
bool motorUpFirst = true;

std::thread api_server_thread;

// REST API
RestAPI api(&currentState, &nextState, &drive, &tableStatus, &arduino, &lidar, &arucoCam1, &strategies);

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
                if (tableStatus.mastStatus) {
                    if(getMapStatus()){ // Getting data from mast
                        tableStatus.updateMapStatus();
                    }else{
                        LOG_ERROR("Failed to get map status from mast");
                        tableStatus.mastStatus = false; // Don't try to get mast information for the rest of the match
                    }
                }
            }
        }

        // Apply the requests received through the REST API
        {
            colorTeam_t requestedColor;
            if (api.consumeColorRequest(requestedColor))
                switchTeamSide(requestedColor);

            std::string requestedStrategy;
            if (api.consumeStrategyRequest(requestedStrategy))
                switchStrategy(requestedStrategy);
        }

        // State machine
        switch (currentState)
        {
        //****************************************************************
        case INIT:
        {
            static bool mast = false;
            if (initState)
            {
                LOG_GREEN_INFO("INIT");
                actuators.disableActuators();
                tableStatus.reset();
                arduino.RGB_Rainbow();
            }
            if(!mast){
                bool sucess;
                mast = StartMat(sucess);
                if(sucess){
                    tableStatus.mastStatus = true;
                    LOG_GREEN_INFO("MAT is ready");
                } 
            }
            if (sensor.readButtonSensor() && !sensor.readLatchSensor() && tableStatus.colorTeam != NONE)
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
                arucoCam1.start();
                arduino.moveMotorDC(80, false);

                if (tableStatus.colorTeam == NONE)
                    arduino.RGB_Blinking(255, 0, 0); // Red Blinking
                tableStatus.calibrationAge = -1;
            }
            
            // colorTeam_t color = readColorSensorSwitch();
            // switchTeamSide(color);

            if (sensor.readLimitSwitchTop() && motorUpFirst){ 
                arduino.moveMotorDC(20,false);
                motorUpFirst = false;
            }
            if (tableStatus.calibrationAge == -1){
                navigation.go();
            } else{
                nextState = CALIBRATION;
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
            tableStatus.pos_opponent.x = 3000.0f;
            tableStatus.pos_opponent.y = 0;
            opponentInAction(tableStatus.pos_opponent);     
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
                arduino.keepMotorDCup();

            }
            bool finished = action.RunFSM();

            if (_millis() > tableStatus.startTime + 100000 || finished || sensor.readButtonSensor())
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
                arduino.keepMotorDCup();
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
                                    { api.start(); });

#ifdef TEST_API_ONLY
    LOG_GREEN_INFO("Running in API test mode only");
    currentState = TEST;
    nextState = TEST;
#else
    currentState = INIT;
    nextState = INIT;
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
    static position_t pos_opponent = {3000,0,0};
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
    arucoCam1.stop();

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

void opponentInAction(position_t position){
    if (position_equals(position, position_t{.x=0, .y=0, .a=0})){
        return; // useless code to get rid of warning 
    }
    // TODO implement this function
    /* Detect from position of adversary the action of the adversary */
}

void switchTeamSide(colorTeam_t color){ // TODO moove to tableState
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
        drive.setCoordinates(pos);
        navigation.goTo(pos, true, true); // Go to starting pos with A* and slow mode to avoid collisions during the switch
    }
}

void check(colorTeam_t color, std::string strategy){
    // Check if the color and strategy are valid
    if (color == NONE)
        LOG_ERROR("Invalid color (", color, ") or strategy (", strategy, ")");
    // TODO add check if necessary
}

void switchStrategy(std::string strategy){ // TODO moove to tableState
    if (currentState == RUN) return;

    if (strategy != action.getStrategy()->getNom()){
        colorTeam_t color = tableStatus.colorTeam;

        check(color, strategy);
        LOG_INFO("Strategy switch detected");
        tableStatus.strategy = strategy;
        position_t pos = action.getStrategy()->StratStartingPos();
        drive.setCoordinates(pos);
    }
}