#pragma once

#include "defs/structs.hpp"
#include "defs/constante.h"
#include <array>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <vector>

class AStar {
    public:
        static const int SCALE = 25;
        static const int ROWS = 2000 / SCALE + 1;
        static const int COLS = 3000 / SCALE + 1;
        static const int CELL_COUNT = ROWS * COLS;
        enum class Layer { Border,
                           Game,
                           Opponent };

        struct Obstacle {
                double x, y;
                double width = 0, height = 0, angle = 0;
                double radius = 0; // A positive radius describes a circle instead of a rectangle.
                bool operator==(const Obstacle&) const = default;
        };

        explicit AStar(double robotRadius = ROBOT_WIDTH / 2.0);
        bool setRobotRadius(double radius);
        bool setObstacles(Layer layer, std::vector<Obstacle> obstacles);
        int findPath(position_t start, position_t goal, std::span<position_t> path, double& length);
        double pathCost(std::span<const position_t> path) const;
        json mapJson(const std::string& knownSession = "",
                     const std::array<uint64_t, 3>& knownVersions = {UINT64_MAX, UINT64_MAX, UINT64_MAX}) const;
        json costmapJson() const;

    private:
        struct Mask {
                std::array<uint8_t, CELL_COUNT> cells{};
                std::vector<Obstacle> obstacles;
                uint64_t version = 0;
                json encoded;
        };
        struct Node {
                double g;
                int parent;
                bool closed;
        };

        void rebuildMask(Mask& mask);
        void encodeMask(Mask& mask);
        double inflation(const Mask& mask) const;
        int cost(int row, int col) const;
        bool pointClear(position_t point) const;
        double lineCost(position_t from, position_t to) const;
        static int cellIndex(position_t position);
        static position_t cellPosition(int index, double angle);

        mutable std::mutex mutex;
        std::array<Mask, 3> masks;
        std::array<Node, CELL_COUNT> nodes;
        std::vector<position_t> waypoints;
        double robotRadius;
        std::string session;
};
