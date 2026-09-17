#pragma once
#include "actions/VirtualAction.hpp"

/*
    Action d'attente simple : temporise pendant WAIT_DURATION_MS
    puis rend la main (FSM_RETURN_DONE) pour permettre au FSM
    de recalculer la meilleure action à effectuer.
*/
class WaitAction : public VirtualAction {
    public:
        WaitAction();
        ~WaitAction() override = default;

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        float available() override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        /*
            Error management and sucess management not implemented because never used but should be
        */
    private:
        static constexpr unsigned long WAIT_DURATION_MS = 500;
        unsigned long startTime;
};