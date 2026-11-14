#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "drive_interface.h"
#include "navigation/navigation.h" // pour nav_return_t & position_t
#include "navigation/pathfind.h"
#include "actions/ActuatorsControl.hpp"
/*
    Action d'attente simple : temporise pendant WAIT_DURATION_MS
    puis rend la main (FSM_RETURN_DONE) pour permettre au FSM
    de recalculer la meilleure action à effectuer.
*/
class DropAction : public VirtualAction {
    public:
        DropAction(TableState* state, DriveControl* drive, ActuatorsControl* actuators);
        ~DropAction() override = default;

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;
        

    protected:
        /*
            Error management and sucess management not used but still implemented for exemple
        */
        bool errorManagement();
        bool successManagement();

    private:
        TableState* tableState;
        DriveControl* drive;
        ActuatorsControl* actuators;

        bool getBestDropZone();
        bool toBestDropPosition();

        typedef enum {
            DROP_STATE_MOVING,
        } dropState_t;

        const position_t DROP_OFFSET = {0, 0, 120};
        position_t target;
        double best_path_length;
        dropState_t state;


};