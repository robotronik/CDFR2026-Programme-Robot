#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"

class Robot;

/*
    Action de retour sur la zone de départ.
    N'est run que si plus rien n'est possible sur la table
    ou si le temps du match est écoulé.
*/
class NavHomeAction : public VirtualAction {
    public:
        NavHomeAction(TableState* tableState, Robot* robot, position_t homePos);
        ~NavHomeAction() override = default;

        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float& reward) override;
        bool fullBlock() override;
        bool movementBlock() override;

    protected:
        bool errorManagement();
        bool successManagement();

    private:
        TableState* tableState;
        Robot* robot;
        // valeur de l'action en point
        int value = 10;
        position_t homePos;
        // Passe à true quand le retour est terminé : l'action n'est alors
        // plus candidate (available() renvoie -1) jusqu'à reset().
        bool done = false;
};