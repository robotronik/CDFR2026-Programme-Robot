#pragma once
#include "actions/ActuatorsControl.hpp"
#include "actions/VirtualAction.hpp"
#include "defs/structs.hpp"
#include "defs/tableState.hpp"
#include "drive_interface.h"

class RobotControl;



class TakeStone : public VirtualAction {
    private:
        TableState* tableState;
        RobotControl* robot;

        enum class TakeStoneState
        {
            MOVE1,
            MOVE2,
        };
        
        const position_t OFFSET_TARGET = {250, 250, 0};
        int value = 23; // TODO change to be max value of points the opponent can make with 3 stones
        TakeStoneState state;
        unsigned long approachStartTime = 0;

        quarry_t* quarry;
        Wall* selected_face;
        position_t target;
        position_t diffpos;

        
        float BestTake(const position_t& from);
        float getBestQuarry(const position_t& from);
        position_t RobotPosSuctionCupOffset(const position_t& stone) const;
        int BestPositionToTake(const position_t& stone, position_t targets[2]) const;//offset pour avoir la bonne face sur la pierre
        position_t RobotTakeOffset(position_t stone, const position_t& from) const;//offset centre/face pierre
        bool resolveTarget();
    public:
        TakeStone(TableState* tableState, RobotControl* robot);
        ~TakeStone() override = default;


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