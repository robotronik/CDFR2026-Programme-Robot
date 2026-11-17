#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/structs.hpp"
#include "drive_interface.h"
#include "navigation/navigation.h" // pour NavReturn & position_t
#include "defs/tableState.hpp"
#include "navigation/pathfind.h"
#include "actions/RobotControl.hpp"

/*
    Action d'attente simple : temporise pendant WAIT_DURATION_MS
    puis rend la main (ReturnFSM::Done) pour permettre au FSM
    de recalculer la meilleure action à effectuer.
*/
class DropAction : public VirtualAction {
    public:
        DropAction(TableState* tableState, RobotControl* robot);
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
        */
        
    private:
        TableState* tableState;
        RobotControl* robot;

        bool getBestDropZone();
        bool toBestDropPosition();
        bool successManagement();

        enum class DropState {
            Moving,
            Releasing,
        };

        Wall* selected_face;
        wallZone_t* selected_drop;
        position_t target;
        double best_path_length;
        DropState state = DropState::Moving;
        unsigned long releaseStartTime = 0;
        static constexpr unsigned long RELEASE_WAIT_MS = 4000;


};