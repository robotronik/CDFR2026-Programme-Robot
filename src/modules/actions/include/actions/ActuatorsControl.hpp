#pragma once
#include "i2c/Arduino.hpp"
#include "navigation/driveControl.h"

class ActuatorsControl {
    public:
        ActuatorsControl(Arduino* arduino, DriveControl* drive);
        bool setPumpState(bool state);
        bool moveServoAndWait(int servo, int target, int speed);
        bool moveColumnsElevator(int level);
        bool homeActuators();
        void enableActuators();
        void disableActuators();

    private:
        Arduino* arduino;
        DriveControl* drive;
};

class Wall {
    private:
        
    public:;
        int nb; // nb de pierres sur la face
        bool wDrop();
        bool wTake();
};

class Tower : public Wall {
    public: 
        bool tDrop(int idx); // idx : index de la pierre à poser
        bool rotate(int idxS, bool dir); // idx S : index de la face/servo à faire tourner, dir : True sens tour False sens mur
}

class Door : public Tower {
    public:
        bool dDrop();
}