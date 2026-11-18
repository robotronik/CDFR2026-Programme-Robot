// Grid-free path planning for a holonomic convex robot.
//
// Pipeline, all exact in continuous space:
//   1. C_obs = obstacle (+) (-robot)      (convex, heading-free)
//   2. optional outward offset by margin
//   3. routing nodes on the union boundary of the C_obs
//   4. lazy visibility graph + A*
//
// Fixed-capacity buffers only: the planner performs no heap allocation, which
// keeps it predictable on an embedded target.
#pragma once

#include <array>

#include "navigation/geom.hpp"

namespace cdfr {

enum class ObstacleKind : uint8_t { Poly,
                                    Circle };

struct ObstacleInput {
        ObstacleKind kind = ObstacleKind::Poly;
        // Poly: `poly` is used. Circle: `center` and `radius` are used.
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

        // Table half-extents. The reference point is kept inside the rectangle
        // eroded by the robot's reach, so paths never leave the table. Set
        // `use_border = false` to plan on an unbounded plane.
        bool use_border = true;
        double table_half_x = 1000.0;
        double table_half_y = 1500.0;
};

struct PlanOutput {
        bool found = false;
        int point_count = 0;
        std::array<Vec, kMaxPathPoints> points{};
        double length = kInf;
        int nodes = 0;
        int edge_tests = 0;
        int expanded = 0;
};

// Plans a path. Returns false when the goal is unreachable.
PlanOutput plan_path(const PlanInput& input);

// Builds the configuration obstacles for a scene (exposed for tests/tools).
// Returns the number of shapes written to `out`.
int build_config_shapes(const PlanInput& input, std::array<Polygon, kMaxShapes>& out);

} // namespace cdfr
