#include "navigation/pathfind.hpp"

#include <cmath>

#include "defs/tableState.hpp"
#include "lidar/Lidar.hpp"
#include "navigation/convexPathfind.hpp"
#include "utils/logger.hpp"

namespace {

using cdfr::ObstacleInput;
using cdfr::ObstacleKind;
using cdfr::PlanInput;
using cdfr::PlanOutput;
using cdfr::Polygon;
using cdfr::Vec;

constexpr double OPPONENT_RADIUS = 225.0; // 450 mm diameter

// A stone is 110 mm wide and 320 mm long. Its stored heading is the long axis
// measured from +x, so at 0 deg it extends 320 mm along y.
constexpr double BLOCK_LENGTH = 320.0;
constexpr double BLOCK_WIDTH = 110.0;

// Corner protected zones: 300 mm along x and 200 mm along y, at every corner of
// the table. The robot may never touch them.
constexpr double CORNER_SIZE_X = 300.0;
constexpr double CORNER_SIZE_Y = 200.0;

// Clearance added to the exact hull. It only keeps the path off a hairline
// contact so the controller has room to correct, nothing more: the heading is
// fixed for a move, so there is no drift to absorb. Kept small, because the
// sharp corners of the real polygon are what allow tight gaps.
constexpr double ROBOT_CLEARANCE = 5.0;

// The robot hull in its own frame: an equilateral triangle with 400 mm sides,
// one vertex pointing forward (+x). One convex part is what lets the planner
// use the exact shape instead of a circle.
// TODO read the real dimensions from a single source of truth.
Polygon buildRobotPolygon() {
    const Vec hull[] = {
        {-250.0, -125.0},
        {150.0, -125.0},
        {250.0, 0.0},
        {150.0, 125.0},
        {-250.0, 125.0},
    };
    return Polygon::from_points(hull, 5);
}

// Kept between calls so a new target does not rebuild the static obstacles.
struct PlannerState {
        PlanInput input;
        PlanOutput last;
        bool hasOpponent = false;
};

PlannerState planner;

void resetInput() {
    planner.input = PlanInput{};
    planner.input.robot = buildRobotPolygon();
    // The robot reference point stays inside the table, so paths never leave
    // the field.
    planner.input.use_border = true;
    planner.input.table_half_x = 1000.0;
    planner.input.table_half_y = 1500.0;
    planner.input.margin = 0.0;
}

bool addObstacle(position_t center, double length, double width, double angleDeg) {
    if (planner.input.obstacle_count >= cdfr::kMaxShapes - 1) {
        LOG_WARNING("Pathfinding: too many obstacles");
        return false;
    }
    ObstacleInput& slot = planner.input.obstacles[planner.input.obstacle_count++];
    slot.kind = ObstacleKind::Poly;
    slot.poly = Polygon::rectangle(center.x, center.y, length, width, angleDeg);
    return true;
}

void addOpponent(Vec center) {
    if (planner.input.obstacle_count >= cdfr::kMaxShapes) return;
    ObstacleInput& slot = planner.input.obstacles[planner.input.obstacle_count++];
    slot.kind = ObstacleKind::Circle;
    slot.center = center;
    slot.radius = OPPONENT_RADIUS;
    planner.hasOpponent = true;
}

// The opponent is always the last obstacle, see addOpponent().
void moveOpponent(Vec center) {
    planner.input.obstacles[planner.input.obstacle_count - 1].center = center;
}

json polygonToJson(const Polygon& poly) {
    json points = json::array();
    for (int i = 0; i < poly.n; i++) {
        points.push_back({{"x", poly.pts[i].x}, {"y", poly.pts[i].y}});
    }
    return points;
}

void removeOpponent() {
    if (planner.input.obstacle_count > 0) planner.input.obstacle_count--;
    planner.hasOpponent = false;
}

} // namespace

int pathfind(position_t start, position_t goal, position_t path[], double& path_length_mm) {
    path_length_mm = 1e6;

    planner.input.start = Vec{start.x, start.y};
    planner.input.goal = Vec{goal.x, goal.y};

    planner.last = cdfr::plan_path(planner.input);
    if (!planner.last.found || planner.last.point_count <= 0) {
        LOG_WARNING("Pathfinding: no path to the target");
        return 0;
    }
    if (planner.last.point_count > PATHFIND_MAX_POINTS) {
        LOG_WARNING("Pathfinding: path has ", planner.last.point_count, " points, too many");
        return 0;
    }

    for (int i = 0; i < planner.last.point_count; i++) {
        path[i] = {planner.last.points[i].x, planner.last.points[i].y, goal.a};
    }
    path_length_mm = planner.last.length;
    return planner.last.point_count;
}

void pathfindSetup(const TableState& tableState) {
    const bool hadOpponent = planner.hasOpponent;
    Vec opponent{0.0, 0.0};
    if (hadOpponent && planner.input.obstacle_count > 0) {
        opponent = planner.input.obstacles[planner.input.obstacle_count - 1].center;
    }

    resetInput();

    // Stones are 320 x 110; the middle quarries store them rotated by 90 degrees.
    for (const auto& quarry : tableState.quarry) {
        if (quarry.nb <= 0) continue;
        const bool rotated = std::fabs(quarry.pos.a - 270.0) < 1.0;
        addObstacle(quarry.pos, rotated ? 110.0 : 320.0, rotated ? 320.0 : 110.0, 0.0);
