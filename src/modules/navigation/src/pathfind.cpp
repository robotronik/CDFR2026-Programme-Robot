#include "navigation/pathfind.hpp"

#include <cmath>

#include "defs/tableState.hpp"
#include "lidar/Lidar.hpp"
#include "navigation/convexPathfind.hpp"
#include "utils/logger.hpp"

namespace {

using planner::ObstacleInput;
using planner::ObstacleKind;
using planner::PlanInput;
using planner::PlanOutput;
using planner::Polygon;
using planner::Vec;

// The opponent is approximated by an octagon inside a 400 x 400 mm footprint:
// rounder than a square, cheaper than a circle, and the planner handles the
// convex polygon exactly.
constexpr double OPPONENT_SIZE = 400.0;

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
    // Circumradius of an equilateral triangle of side s is s / sqrt(3).
    constexpr double kSide = 400.0;
    constexpr double kR = kSide / 1.7320508075688772;  // s / sqrt(3)
    const Vec hull[] = {
        {kR, 0.0},                                         // nose, forward
        {-kR / 2.0, kSide / 2.0},                          // rear left
        {-kR / 2.0, -kSide / 2.0},                         // rear right
    };
    return Polygon::from_points(hull, 3);
}

struct PlannerState {
        PlanInput input;
        PlanOutput last;
        // Table the obstacles are rebuilt from on every plan. Null until
        // pathfindSetup() has been called.
        const TableState* table = nullptr;
        // Index of the opponent in `input.obstacles`, or -1. Stored explicitly
        // instead of assuming it is the last slot: obstacles can be added
        // before or after the opponent, and writing to the wrong slot silently
        // turned a stone into the opponent (or the reverse).
        int opponentIndex = -1;
};

PlannerState state;

// Rebuilds the static obstacles from state.table, carrying the opponent over.
void rebuildObstacles();

void resetInput() {
    state.input = PlanInput{};
    state.opponentIndex = -1;
    state.input.robot = buildRobotPolygon();
    // The robot reference point stays inside the table, so paths never leave
    // the field.
    state.input.use_border = true;
    state.input.table_half_x = 1000.0;
    state.input.table_half_y = 1500.0;
    state.input.margin = 0.0;
}

// Places a rectangle of `x_extent` by `y_extent` (before rotation) at `center`.
// Both are full side lengths, so the caller never has to reason about which one
// the polygon builder takes first.
bool addObstacle(position_t center, double x_extent, double y_extent, double angleDeg) {
    if (state.input.obstacle_count >= planner::kMaxShapes - 1) {
        LOG_WARNING("Pathfinding: too many obstacles");
        return false;
    }
    ObstacleInput& slot = state.input.obstacles[state.input.obstacle_count++];
    slot.kind = ObstacleKind::Poly;
    slot.poly = Polygon::rectangle(center.x, center.y, x_extent, y_extent, angleDeg);
    return true;
}

Polygon buildOpponentPolygon(Vec center) {
    constexpr double kHalf = OPPONENT_SIZE / 2.0;
    // Half-side of the corner cut: kHalf * tan(22.5 deg).
    constexpr double kCut = kHalf * 0.41421356237309503;
    const Vec octagon[] = {
        {kHalf, kCut},   {kCut, kHalf},   {-kCut, kHalf},   {-kHalf, kCut},
        {-kHalf, -kCut}, {-kCut, -kHalf}, {kCut, -kHalf},   {kHalf, -kCut},
    };
    Vec moved[8];
    for (int i = 0; i < 8; i++) moved[i] = center + octagon[i];
    return Polygon::from_points(moved, 8);
}

void addOpponent(Vec center) {
    if (state.input.obstacle_count >= planner::kMaxShapes - 1) {
        LOG_WARNING("Pathfinding: no room for the opponent");
        return;
    }
    ObstacleInput& slot = state.input.obstacles[state.input.obstacle_count];
    slot.kind = ObstacleKind::Poly;
    slot.poly = buildOpponentPolygon(center);
    // rebuildObstacles() carries the opponent over by reading this back, so it
    // has to be set for the polygon opponent too, not just circles.
    slot.center = center;
    state.opponentIndex = state.input.obstacle_count++;
}

void moveOpponent(Vec center) {
    if (state.opponentIndex < 0) return;
    ObstacleInput& slot = state.input.obstacles[state.opponentIndex];
    slot.poly = buildOpponentPolygon(center);
    slot.center = center;
}

json polygonToJson(const Polygon& poly) {
    json points = json::array();
    for (int i = 0; i < poly.n; i++) {
        points.push_back({{"x", poly.pts[i].x}, {"y", poly.pts[i].y}});
    }
    return points;
}

// Removes the opponent by moving the last obstacle into its slot: no other
// entry shifts, and the array stays contiguous.
void removeOpponent() {
    if (state.opponentIndex < 0) return;
    const int last = state.input.obstacle_count - 1;
    if (state.opponentIndex != last) {
        state.input.obstacles[state.opponentIndex] = state.input.obstacles[last];
    }
    state.input.obstacle_count = last;
    state.opponentIndex = -1;
}

// Four protected rectangles, one per table corner. Each is CORNER_SIZE_X wide
// (along x) and CORNER_SIZE_Y deep (along y), flush with the corner.
void addCornerZones() {
    const double halfX = state.input.table_half_x;
    const double halfY = state.input.table_half_y;
    const struct { double x, y; } corners[4] = {
        {-1.0, -1.0}, {-1.0, 1.0}, {1.0, -1.0}, {1.0, 1.0}};
    for (const auto& c : corners) {
        position_t zone;
        zone.x = c.x * (halfX - CORNER_SIZE_X / 2.0);
        zone.y = c.y * (halfY - CORNER_SIZE_Y / 2.0);
        zone.a = 0.0;
        addObstacle(zone, CORNER_SIZE_X, CORNER_SIZE_Y, 0.0);
    }
}

void rebuildObstacles() {
    if (state.table == nullptr) return;
    const TableState& tableState = *state.table;
    const bool hadOpponent = state.opponentIndex >= 0;
    Vec opponent{0.0, 0.0};
    if (hadOpponent) opponent = state.input.obstacles[state.opponentIndex].center;

    resetInput();

    // Stones are 110 across and 320 long, and their stored heading is the real
    // rotation, so it is passed straight through. Guessing the orientation from
    // the angle put every stone a quarter turn off from where it actually sits.
    for (const auto& quarry : tableState.quarry) {
        if (quarry.nb <= 0) continue;
        // A stone lies across y at 0 deg: 110 wide, 320 long.
        addObstacle(quarry.pos, BLOCK_WIDTH, BLOCK_LENGTH, quarry.pos.a);
    }

    // Wall areas lie along their facing direction.
    for (const auto& zone : tableState.wallZone) {
        if (zone.nb <= 0) continue;
        addObstacle(zone.pos, BLOCK_WIDTH, BLOCK_LENGTH, zone.pos.a);
    }

    addCornerZones();

    if (hadOpponent) addOpponent(opponent);
}

} // namespace

int pathfind(position_t start, position_t goal, position_t path[], double& path_length_mm) {
    path_length_mm = 1e6;

    // Rebuild the obstacles from the table on every plan. Stones are picked up
    // and dropped, so a cached set goes stale the moment a quarry's count
    // changes; rebuilding is a handful of small polygon constructions and costs
    // far less than the A* that follows.
    rebuildObstacles();

    state.input.start = Vec{start.x, start.y};
    state.input.goal = Vec{goal.x, goal.y};

    // Plan with the exact hull at the goal heading.
    //
    // The drive controller steers towards goal.a, so planning from the live
    // heading made the path a function of the very angle the controller was
    // changing: the path jumped sides and the robot oscillated. Planning at
    // goal.a fixes that, and the exact hull is what lets the robot hug a wall
    // or reach into a quarry.
    //
    // The plan is only valid while the robot stays near goal.a, which it does
    // because it turns towards that heading as it drives.
    state.input.robot = planner::rotate_polygon(buildRobotPolygon(), goal.a);
    // inflate_polygon, not grow_polygon: the arc-sampled offset would turn the
    // 3-vertex hull into ~33 vertices and triple the planner's edge tests, and
    // rounding the corners defeats the polygon.
    state.input.robot = planner::inflate_polygon(state.input.robot, ROBOT_CLEARANCE);
    state.last = planner::plan_path(state.input);

    if (!state.last.found || state.last.point_count <= 0) {
        LOG_WARNING("Pathfinding: no path to the target");
        return 0;
    }
    if (state.last.point_count > PATHFIND_MAX_POINTS) {
        LOG_WARNING("Pathfinding: path has ", state.last.point_count, " points, too many");
        return 0;
    }

    for (int i = 0; i < state.last.point_count; i++) {
        path[i] = {state.last.points[i].x, state.last.points[i].y, goal.a};
    }
    path_length_mm = state.last.length;
    return state.last.point_count;
}

void pathfindSetup(const TableState& tableState) {
    // Remember the table so pathfind() can rebuild from it every call.
    state.table = &tableState;
    rebuildObstacles();
}

void pathfindUpdateOpponent(Lidar* lidar, const TableState& tableState) {
    if (lidar == nullptr) return;

    // The lidar pipeline already tracks the opponent, so reuse it rather than
    // clustering the raw scan a second time.
    const position_t pos = tableState.pos_opponent;
    const bool known = pos.x < 2000.0; // OPPONENT_POS_UNKNOWN is (3000, 0)

    if (!known) {
        removeOpponent();
        return;
    }

    const Vec center{pos.x, pos.y};
    if (state.opponentIndex >= 0) {
        moveOpponent(center);
    } else {
        addOpponent(center);
    }
}

void pathfindToJson(json& out) {
    out = json::object();

    // Raw obstacles, as physical objects on the table. `kind` tells the
    // opponent apart from the stones for any consumer that wants to style them.
    json obstacles = json::array();
    for (int i = 0; i < state.input.obstacle_count; i++) {
        const ObstacleInput& ob = state.input.obstacles[i];
        const bool isOpponent = (i == state.opponentIndex);
        obstacles.push_back({{"kind", isOpponent ? "opponent" : "stone"},
                             {"corners", polygonToJson(ob.poly)}});
    }
    out["obstacles"] = obstacles;

    // The robot hull in its own frame, unrotated. The planner inflates a
    // rotated copy, but the frontend places this one at `pos` and rotates it by
    // `a` itself, so sending the rotated version would apply the heading twice.
    out["robot"] = polygonToJson(buildRobotPolygon());

    // The inflated obstacles the planner actually avoids: C_obs = O (+) (-robot).
    // Drawn as a translucent layer to show why a path keeps its distance.
    //
    // build_config_shapes() emits the four table-border walls first, then one
    // shape per obstacle in order. The border is flagged so the frontend can
    // draw it as a thin edge instead of filling the whole table, and the
    // opponent's shape is flagged so it can be tinted.
    std::array<Polygon, planner::kMaxShapes> shapes{};
    const int shapeCount = planner::build_config_shapes(state.input, shapes);
    const int borderCount = state.input.use_border ? 4 : 0;
    json inflated = json::array();
    for (int i = 0; i < shapeCount; i++) {
        // Shape order matches the obstacle order after the border block. The
        // `>= 0` guard matters: for border shapes the index is negative, and
        // with no opponent (-1) a missing guard flagged a wall as the opponent.
        const int obstacleIndex = i - borderCount;
        const bool isOpponent =
            (state.opponentIndex >= 0 && obstacleIndex == state.opponentIndex);
        inflated.push_back({{"corners", polygonToJson(shapes[i])},
                            {"border", i < borderCount},
                            {"opponent", isOpponent}});
    }
    out["inflated"] = inflated;

    json path = json::array();
    for (int i = 0; i < state.last.point_count; i++) {
        path.push_back({{"x", state.last.points[i].x}, {"y", state.last.points[i].y}});
    }
    out["path"] = path;
    out["found"] = state.last.found;
    out["length_mm"] = state.last.found ? state.last.length : 0.0;
}
