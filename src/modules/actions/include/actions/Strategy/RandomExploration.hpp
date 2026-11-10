#pragma once

#include <optional>
#include <random>

#include "actions/ElementalAction/GoToPositionAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "defs/constante.h"
#include "defs/tableState.hpp"

class RandomExploration : public VirtualStrategy {
public:
    RandomExploration(DriveControl& drive, const TableState& table) : drive(drive), table(table) {
        nom = "ExplorationAleatoire";
    }

    position_t StratStartingPos() override {
        position_t pos = START_POSITION;
        if (table.colorTeam == YELLOW)
            position_robot_flip(pos);
        return pos;
    }

    VirtualAction* bestAction() override {
        if (!status) return nullptr;

        // Leave room for the robot and the border obstacles on the A* grid.
        constexpr int margin = ROBOT_WIDTH / 2 + 50;
        std::uniform_real_distribution<double> x(-MAX_WIDTH_TABLE + margin, MAX_WIDTH_TABLE - margin);
        std::uniform_real_distribution<double> y(-MAX_LENGTH_TABLE + margin, MAX_LENGTH_TABLE - margin);
        const auto from = drive.getPosition();
        const position_t target{x(random), y(random), from.a};
        if (position_distance(from, target) < ROBOT_WIDTH) return nullptr;

        destination.emplace("Explore", target, &drive);
        return &*destination;
    }

    VirtualAction* tempAction() override { return nullptr; }

private:
    DriveControl& drive;
    const TableState& table;
    std::mt19937 random{std::random_device{}()};
    std::optional<GoToPositionAction> destination;
};
