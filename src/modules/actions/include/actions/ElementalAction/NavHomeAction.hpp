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
        // the action's value in points
        int value = 10;
        position_t homePos;
        // Set to true when the return is complete: the action is then no longer
        // a candidate (available() returns -1) until reset().
        bool done = false;
};