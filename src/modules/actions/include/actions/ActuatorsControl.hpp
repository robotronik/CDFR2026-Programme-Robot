#pragma once
#include "drive_interface.h"
#include <cstddef>

class Arduino;
class DriveControl;

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

class Wall {
    public:
        typedef struct{
            unsigned short int pumpIdx;
            unsigned short int electroIdx[3];
            unsigned short int servoIdx[3]; 
        } actuatorsIdx_t;

        Wall(actuatorsIdx_t idx, position_t offset, ActuatorsControl* actuator)
            : idx(idx), actuator(actuator), offset(offset) { // Modifier la synthaxe
        };
        bool stone[3] = {false, false, false}; // number of stones on the face
        void setStone(int index, bool value){ stone[index] = value; };
        void voidStone(){ for (int i = 0; i < 3; i++) stone[i] = false; };
        void fullStone(){ for (int i = 0; i < 3; i++) stone[i] = true; };
        bool hasStone(){ for (int i = 0; i < 3; i++) if (stone[i]) return true; return false; };

        bool wDrop();
        bool wTake();
        bool wEnableElectro();
        bool wDisableElectro();
        bool DeployBackGripper(bool state);
        position_t getOffset() const { return offset; }
        bool home();

    protected:
        actuatorsIdx_t idx; // struct containing all the pins for servo pum electro
        ActuatorsControl* actuator;
        position_t offset;
};

class Tower : public Wall {
    public:
        Tower(actuatorsIdx_t idx, position_t offset, ActuatorsControl* actuator)
            : Wall(idx, offset, actuator) {
        };
        bool tDrop(int idx);
        bool rotate(int position);

    protected:
        //nothing to store for now
};

class Gate : public Tower {
    private:
        int dropStep = 0;
        position_t retreatTarget;

    public:
        Gate(actuatorsIdx_t idx, position_t offset, ActuatorsControl* actuator)
            : Tower(idx, offset, actuator) {
        };
        bool gDrop(DriveControl* drive);
        bool gBlockFlip(int position);
};
