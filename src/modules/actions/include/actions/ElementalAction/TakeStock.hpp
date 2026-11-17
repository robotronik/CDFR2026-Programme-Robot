#pragma once
#include "actions/ActuatorsControl.hpp"
#include "actions/VirtualAction.hpp"
#include "defs/structs.hpp"
#include "defs/tableState.hpp"
#include "drive_interface.h"

class RobotControl;



class TakeStock : public VirtualAction {
    private:
        TableState* tableState;
        RobotControl* robot;

        enum class TakeStockState
        {
            MOVE1,
            MOVE2,
        };
        
        const position_t OFFSET_TARGET = {250, 250, 0};
        int value = 23; // TODO change to be max value of points adv can make with 3 rocks
        TakeStockState state;
        unsigned long approachStartTime = 0;

        quarry_t* quarry;
        Wall* selected_face;
        position_t target;
        position_t diffpos;

        
        float BestTake(const position_t& from);
        float getBestQuarry(const position_t& from);
        position_t getFacePosition(const position_t& stock) const;
        int BestPositionToTake(const position_t& stock, position_t targets[2]) const;
        bool resolveTarget();
    public:
        TakeStock(TableState* tableState, RobotControl* robot);
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