#pragma once
#include "actions/VirtualAction.hpp"
#include "defs/tableState.hpp"
#include "drive_interface.h"
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

        const position_t OFFSET_TARGET = {150, 150, 0};
        int value = 23; // TODO change to be max value of points adv can make with 3 rocks
        takeStockState_t state;

        position_t target;
        position_t diffpos;

        float BestTake(const position_t& from);
        float getBestQuarry(const position_t& from);
        void BestPositionToTake(const position_t&  from);
        bool resolveTarget();
    public:
        TakeStock(TableState* tableState);
        ~TakeStock() override = default;


        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        bool errorManagement();
        bool successManagement();
};