#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/constante.h"
#include "defs/structs.hpp"
#include "utils/utils.h"
#include "drive_interface.h"
#include "navigation/navigation.h" // pour NavReturn & position_t
#include "defs/tableState.hpp"
#include "navigation/pathfind.h"
#include "actions/RobotControl.hpp"
#include <cmath>

/*
    Action d'attente simple : temporise pendant WAIT_DURATION_MS
    puis rend la main (ReturnFSM::Done) pour permettre au FSM
    de recalculer la meilleure action à effectuer.
*/
class DropAction : public VirtualAction {
    public:
        DropAction(TableState* state, DriveControl* drive, RobotControl* robot);
        ~DropAction() override = default;

        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;
        

    protected:
        /*
            Error management and sucess management not used but still could bin in the future
            bool errorManagement();
            bool successManagement();
        */
        
    private:
        TableState* tableState;
        DriveControl* drive;
        RobotControl* robot;

        bool getBestDropZone();
        bool toBestDropPosition();

        enum class DropState {
            Moving,
        };

        const position_t DROP_OFFSET = {0, 0, 120};
        position_t target;
        double best_path_length;
        DropState state;


};