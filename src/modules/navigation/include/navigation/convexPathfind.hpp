// Grid-free path planning for a holonomic convex robot, exact in continuous space.
#pragma once

#include <array>

#include "navigation/geom.hpp"

namespace planner {

enum class ObstacleKind : uint8_t { Poly,
                                    Circle };

struct ObstacleInput {
        ObstacleKind kind = ObstacleKind::Poly;
        // Poly uses `poly`; Circle uses `center` and `radius`.
        Polygon poly{};
        Vec center{};
        double radius = 0.0;
};

struct PlanInput {
        Vec start{};
        Vec goal{};
        Polygon robot{};
        int obstacle_count = 0;
        std::array<ObstacleInput, kMaxShapes> obstacles{};
        double margin = 0.0;

        // Set use_border = false to plan on an unbounded plane.
        bool use_border = true;
        double table_half_x = 1000.0;
        double table_half_y = 1500.0;
};

struct PlanOutput {
        bool found = false;
        int point_count = 0;
        std::array<Vec, kMaxPathPoints> points{};
        double length = kInf;
        // The goal lies inside a configuration obstacle, so it cannot be reached.
        bool goal_blocked = false;
};

// Plans a path, or found = false when the goal is unreachable.
PlanOutput plan_path(const PlanInput& input);

// Configuration obstacles for a scene. Returns the number written to `out`.
int build_config_shapes(const PlanInput& input, std::array<Polygon, kMaxShapes>& out);

} // namespace planner
