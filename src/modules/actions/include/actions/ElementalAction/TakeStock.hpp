#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h"

typedef enum
{
    MOVE1 =0x0,
    MOVE2 =0x1,
} takeStockState_t;

class TakeStock : public VirtualAction {
    private:
        TableState* tableState;
        DriveControl* drive;
        position_t target;
        bool moving;
        bool needsTarget = false;
        bool done = false;
        int value;
        takeStockState_t state;
        bool resolveTarget();
    public:
        TakeStock(TableState* tableState);
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