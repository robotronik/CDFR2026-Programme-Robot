#pragma once
#include "actions/VirtualAction.hpp"
#include "navigation/driveControl.h"
#include "defs/tableState.hpp"
/*
    Action de retour sur la zone de départ.
    N'est run que si plus rien n'est possible sur la table
    ou si le temps du match est écoulé.
*/
class NavHomeAction : public VirtualAction {
    public:
        NavHomeAction(TableState* tableState, DriveControl* drive);
        ~NavHomeAction() override = default;

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        float available() override;
        bool fullBlock() override;
        bool mouvementBlock() override;
    protected:
        bool errorManagement();
        bool successManagement();
    private:
        position_t homePos;
        TableState* tableState;
        DriveControl* drive;

};