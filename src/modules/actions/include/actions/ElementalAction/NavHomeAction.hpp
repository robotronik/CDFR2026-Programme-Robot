#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"

class RobotControl;

/*
    Action de retour sur la zone de départ.
    N'est run que si plus rien n'est possible sur la table
    ou si le temps du match est écoulé.
*/
class NavHomeAction : public VirtualAction {
    public:
        NavHomeAction(TableState* tableState, RobotControl* robot);
        ~NavHomeAction() override = default;

        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;
    protected:
        bool errorManagement();
        bool successManagement();
    private:
        position_t homePos;
        TableState* tableState;
        RobotControl* robot;
        // valeur de l'action en point
        int value = 10;
        // Passe à true quand le retour est terminé : l'action n'est alors
        // plus candidate (available() renvoie -1) jusqu'à reset().
        bool done = false;
};