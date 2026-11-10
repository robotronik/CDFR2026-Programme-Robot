#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "actions/ElementalAction/GoToPositionAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "defs/constante.h"
#include "defs/tableState.hpp"

class StocksEtChateau : public VirtualStrategy {
public:
    StocksEtChateau(DriveControl& drive, const TableState& table) : drive(drive), table(table) {
        nom = "StocksEtChateau";
    }

    position_t StratStartingPos() override {
        position_t pos = START_POSITION;
        if (table.colorTeam == YELLOW)
            position_robot_flip(pos);
        return pos;
    }

    VirtualAction* bestAction() override {
        if (!status || next > stocks.size()) return nullptr;
        if (destination && !destination->completed()) {
            stop();
            return nullptr;
        }

        if (next == stocks.size()) {
            ++next;
            return &destination.emplace("Chateau", StratStartingPos(), &drive);
        }

        auto target = stocks[next++];
        if (table.colorTeam == YELLOW)
            position_robot_flip(target);
        return &destination.emplace("Stock " + std::to_string(next), target, &drive);
    }

    VirtualAction* tempAction() override { return nullptr; }
    void reset() override {
        next = 0;
        destination.reset();
    }

private:
    // Match background.svg. Border targets are set inward to leave room for the robot.
    static constexpr std::array<position_t, 10> stocks{{
        {-675, 750, 180}, {-675, 250, 180}, {-500, 0, 180},
        {-675, -250, 180}, {-675, -750, 180}, {675, -750, 0},
        {675, -250, 0}, {500, 0, 0}, {675, 250, 0}, {675, 750, 0}
    }};

    DriveControl& drive;
    const TableState& table;
    std::size_t next = 0;
    std::optional<GoToPositionAction> destination;
};
