#include "navigation/pathfind.h"
#include "navigation/astar.h"
#include "navigation/navigation.h"
#include "defs/constante.h"
#include "defs/tableState.hpp"
#include "lidar/Lidar.hpp"
#include <cmath>

int pathfind(position_t start, position_t goal, std::span<position_t> path, double& pathLengthMm) {
    navigation.updatePathfinder();
    return navigation.getPathfinder().findPath(start, goal, path, pathLengthMm);
}

void updateGameObstacles(AStar& planner, const TableState& tableState) {
    std::vector<AStar::Obstacle> obstacles;
    for (const quarry_t& stone : tableState.quarry) {
        if (stone.nb > 0) obstacles.push_back({stone.pos.x, stone.pos.y, GAME_ELEMENT_DEPTH_MM, GAME_ELEMENT_WIDTH_MM, stone.pos.a});
    }
    for (const wallZone_t& zone : tableState.wallZone) {
        if (zone.nb > 0) obstacles.push_back({zone.pos.x, zone.pos.y, GAME_ELEMENT_DEPTH_MM, GAME_ELEMENT_WIDTH_MM, zone.pos.a});
    }
    planner.setObstacles(AStar::Layer::Game, std::move(obstacles));
}

void updateOpponentObstacles(AStar& planner, const Lidar& lidar) {
    std::vector<AStar::Obstacle> obstacles;
    std::array<bool, AStar::CELL_COUNT> occupied{};
    for (int i = 0; i < lidar.count; ++i) {
        const lidarAnalize_t& point = lidar.data[i];
        if (!point.onTable || !std::isfinite(point.x) || !std::isfinite(point.y) ||
            point.x < -1000 || point.x > 1000 || point.y < -1500 || point.y > 1500) continue;
        const int row = int(std::round((point.x + 1000) / AStar::SCALE));
        const int col = int(std::round((point.y + 1500) / AStar::SCALE));
        if (occupied[row * AStar::COLS + col]) continue;
        occupied[row * AStar::COLS + col] = true;
    }
    // Quantise once per scan, with a margin covering every point in the occupied cell.
    const double radius = OPPONENT_ROBOT_RADIUS + AStar::SCALE * std::sqrt(2.0) / 2;
    for (int index = 0; index < AStar::CELL_COUNT; ++index) {
        if (occupied[index]) obstacles.push_back({double(index / AStar::COLS * AStar::SCALE - 1000),
                                                  double(index % AStar::COLS * AStar::SCALE - 1500), 0, 0, 0, radius});
    }
    planner.setObstacles(AStar::Layer::Opponent, std::move(obstacles));
}
