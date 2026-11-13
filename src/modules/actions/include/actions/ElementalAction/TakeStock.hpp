#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h"

class TakeStock : public VirtualAction {
    private:
        TableState* tableState;
        DriveControl* drive;
        position_t target;
        bool moving;
        bool needsTarget = false;
        bool done = false;
        int value;
        bool resolveTarget();
    public:
        TakeStock(const std::string& name, TableState* tableState);
        ~TakeStock() override = default;

        position_t BestPositionToTake(const position_t&  from, position_t& target);

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        bool errorManagement();
        bool successManagement();
        bool BestTake(const position_t& from, position_t& target);
        bool getBestQuarry(const position_t& from, position_t& target);
};