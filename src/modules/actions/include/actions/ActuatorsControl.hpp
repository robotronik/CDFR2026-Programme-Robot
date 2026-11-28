#pragma once
#include "drive_interface.h"

class Arduino;
class DriveControl;
class Navigation;

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
        DriveControl* getDrive() { return drive; }

    private:
        Arduino* arduino;
        DriveControl* drive;
};

class Wall {
    public:
        typedef struct {
                unsigned short int pumpIdx;
                unsigned short int electroIdx[3];
                unsigned short int servoIdx[3];
        } actuatorsIdx_t;

        // TODO: fix the syntax
        Wall(actuatorsIdx_t idx, position_t offset, ActuatorsControl* actuator)
            : idx(idx), actuator(actuator), offset(offset) {}
        bool stones[3] = {false, false, false}; // stones present on the face (true = present)
        void setStone(int index, bool value) { stones[index] = value; };
        void clearStones() {
            for (int i = 0; i < 3; i++)
                stones[i] = false;
        };
        void fillStones() {
            for (int i = 0; i < 3; i++)
                stones[i] = true;
        };
        bool hasAnyStone() {
            for (int i = 0; i < 3; i++)
                if (stones[i]) return true;
            return false;
        };

        bool releaseStone();
        bool grabStone();
        bool enableElectro();
        bool disableElectro();
        bool deployBackGripper(bool state);
        position_t getOffset() const { return offset; }
        void setNavigation(Navigation* navigation) { navigation_ = navigation; }
        virtual bool home();

    protected:
        actuatorsIdx_t idx; // pins for the servo, pump and electro of each face
        ActuatorsControl* actuator;
        position_t offset;
        Navigation* navigation_ = nullptr; // for the gate's navigation sequence
};

class Tower : public Wall {
    public:
        Tower(actuatorsIdx_t idx, position_t offset, ActuatorsControl* actuator)
            : Wall(idx, offset, actuator) {}
        bool dropStone(int idx);
        bool rotate(int position);
        bool home() override;

    protected:
        // Nothing to store for now
};

class Gate : public Tower {
    public:
        Gate(actuatorsIdx_t idx, position_t offset, ActuatorsControl* actuator)
            : Tower(idx, offset, actuator) {}
        bool home() override;
        bool runDropSequence();
        bool flipBlock(int position);

    private:
        int dropStep = 0;
        position_t retreatTarget;
};
