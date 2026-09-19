#pragma once
#include "actions/VirtualAction.hpp"
#include "navigation/navigation.h"
#include "main.hpp"

/*
    Action de retour sur la zone de départ.
    N'est run que si plus rien n'est possible sur la table
    ou si le temps du match est écoulé.
*/
class NavHomeAction : public VirtualAction {
    public:
        NavHomeAction();
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

};