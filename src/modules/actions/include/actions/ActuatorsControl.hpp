#pragma once
#include "i2c/Arduino.hpp"
#include "navigation/driveControl.h"

class ActuatorsControl {
    public:
        ActuatorsControl(Arduino* arduino, DriveControl* drive);
        bool setPumpState(bool state, int pumpIdx);
        bool setElectroState(bool state, int electroIdx);
        bool moveServoAndWait(int servo, int target, int speed);
        bool moveColumnsElevator(int level);
        bool homeActuators();
        void enableActuators();
        void disableActuators();

    private:
        Arduino* arduino;
        DriveControl* drive;
};

class Wall{
    public:
        Wall(int idx[3], position_t offset, ActuatorsControl* actuator)
        : actuator(actuator), offset(offset){ // Modifier la synthaxe
            pumpIdx[0] = idx[0]; 
            pumpIdx[1] = idx[1];
            pumpIdx[2] = idx[2];
        };
        bool wDrop();
        bool wTake();
    protected:
        int pumpIdx[3]; // index of the pump for the face
        ActuatorsControl* actuator;
        position_t offset;
};

class Tower : public Wall {
    public: 
        Tower(int idxS, int idx[3], position_t offset, ActuatorsControl* actuator) 
            : Wall(idx, offset, actuator) {
            servo = idxS;
        };
        bool tDrop(int idx);
        bool rotate(bool dir); 
    protected:
        int servo;
        
};

class Door : public Tower {
    private:
        int servoD;
    public:
        Door(int idxSD, int idxS, int idx[3], position_t offset, ActuatorsControl* actuator) 
            : Tower (idxS, idx, offset, actuator) {
            servoD = idxSD;
        };
        bool dDrop();
        bool flip();
};