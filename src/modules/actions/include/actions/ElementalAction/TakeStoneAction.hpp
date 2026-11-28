#pragma once
#include "actions/ActuatorsControl.hpp"
#include "actions/VirtualAction.hpp"
#include "defs/structs.hpp"
#include "defs/tableState.hpp"
#include "drive_interface.h"

class Robot;

class TakeStoneAction : public VirtualAction {
    private:
        TableState* tableState;
        Robot* robot;

        enum class TakeStoneState {
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
        position_t diffPos;

        float bestTake(const position_t& from);
        float getBestQuarry(const position_t& from);
        position_t robotPosSuctionCupOffset(const position_t& stone) const;
        int bestPositionToTake(const position_t& stone, position_t targets[2]) const; // offset to line the correct face up with the stone
        position_t robotTakeOffset(position_t stone, const position_t& from) const; // offset from the stone centre to its face
        Wall* getBestPossibleFace();

    public:
        TakeStoneAction(TableState* tableState, Robot* robot);
        ~TakeStoneAction() override = default;

        ReturnFSM run() override;
        bool stop() override;
        void reset() override;
        bool available(float& reward) override;
        bool fullBlock() override;
        bool movementBlock() override;

    protected:
        bool errorManagement();
        bool successManagement();
};