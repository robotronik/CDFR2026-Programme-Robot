#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/structs.hpp"
#include "drive_interface.h"
#include "navigation/navigation.h" // pour NavReturn & position_t
#include "defs/tableState.hpp"
#include "navigation/pathfind.h"
#include "actions/RobotControl.hpp"

/*
    Action de pose : pose les pierres portees dans une zone de construction
    libre (wall area) de l'equipe, puis rend la main.
*/
class DropStoneAction : public VirtualAction {
    public:
        DropStoneAction(TableState* tableState, RobotControl* robot);
        ~DropStoneAction() override = default;

        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float& reward) override;
        bool fullBlock() override;
        bool movementBlock() override;

    protected:
        /*
            Error management and sucess management not used but still could bin in the future
            bool errorManagement();
        */

    private:
        TableState* tableState;
        RobotControl* robot;

        bool findBestConstructionZone();
        bool moveTargetToConstructionPosition();
        bool successManagement();

        enum class DropStoneState {
            Moving,
            Releasing,
        };

        Wall* selected_face;
        wallZone_t* selected_zone;
        position_t target;
        double best_path_length;
        DropStoneState state = DropStoneState::Moving;
        unsigned long releaseStartTime = 0;
        static constexpr unsigned long RELEASE_WAIT_MS = 200;
};