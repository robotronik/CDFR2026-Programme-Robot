#include "navigation/pathfind.hpp"

#include <cmath>

#include "defs/tableState.hpp"
#include "lidar/Lidar.hpp"
#include "navigation/convexPathfind.hpp"
#include "utils/logger.hpp"

namespace {

using nav::ObstacleInput;
using nav::ObstacleKind;
using nav::PlanInput;
using nav::PlanOutput;
using nav::Polygon;
using nav::Vec;

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

// Clearance added to the exact hull. The path is replanned continuously at the
// live heading, so this does not absorb heading drift: it only keeps the path
// off a hairline contact so the controller has room to correct. Kept small,
// because the sharp corners of the real polygon are what allow tight gaps.
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

PlannerState planner;

// Rebuilds the static obstacles from planner.table, carrying the opponent over.
void rebuildObstacles();

void resetInput() {
    planner.input = PlanInput{};
    planner.opponentIndex = -1;
    planner.input.robot = buildRobotPolygon();
    // The robot reference point stays inside the table, so paths never leave
    // the field.
    planner.input.use_border = true;
    planner.input.table_half_x = 1000.0;
    planner.input.table_half_y = 1500.0;
    planner.input.margin = 0.0;
}

bool addObstacle(position_t center, double length, double width, double angleDeg) {
    if (planner.input.obstacle_count >= nav::kMaxShapes - 1) {
        LOG_WARNING("Pathfinding: too many obstacles");
        return false;
    }
    ObstacleInput& slot = planner.input.obstacles[planner.input.obstacle_count++];
    slot.kind = ObstacleKind::Poly;
    // Polygon::rectangle takes the x extent first, and `length` is the long
    // axis, so the block lies across y at 0 deg.
    slot.poly = Polygon::rectangle(center.x, center.y, width, length, angleDeg);
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
    if (planner.input.obstacle_count >= nav::kMaxShapes) return;
    ObstacleInput& slot = planner.input.obstacles[planner.input.obstacle_count];
    slot.kind = ObstacleKind::Poly;
    slot.poly = buildOpponentPolygon(center);
    // rebuildObstacles() carries the opponent over by reading this back, so it
    // has to be set for the polygon opponent too, not just circles.
    slot.center = center;
    planner.opponentIndex = planner.input.obstacle_count++;
}

void moveOpponent(Vec center) {
    if (planner.opponentIndex < 0) return;
    ObstacleInput& slot = planner.input.obstacles[planner.opponentIndex];
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
    if (planner.opponentIndex < 0) return;
    const int last = planner.input.obstacle_count - 1;
    if (planner.opponentIndex != last) {
        planner.input.obstacles[planner.opponentIndex] = planner.input.obstacles[last];
    }
    planner.input.obstacle_count = last;
    planner.opponentIndex = -1;
}

// Four protected rectangles, one per table corner. Each is CORNER_SIZE_X wide
// (along x) and CORNER_SIZE_Y deep (along y), flush with the corner.
void addCornerZones() {
    const double halfX = planner.input.table_half_x;
    const double halfY = planner.input.table_half_y;
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
    if (planner.table == nullptr) return;
    const TableState& tableState = *planner.table;
    const bool hadOpponent = planner.opponentIndex >= 0;
    Vec opponent{0.0, 0.0};
    if (hadOpponent) opponent = planner.input.obstacles[planner.opponentIndex].center;

    resetInput();

    // Stones are 110 across and 320 long, and their stored heading is the real
    // rotation, so it is passed straight through. Guessing the orientation from
    // the angle put every stone a quarter turn off from where it actually sits.
    for (const auto& quarry : tableState.quarry) {
        if (quarry.nb <= 0) continue;
        addObstacle(quarry.pos, BLOCK_LENGTH, BLOCK_WIDTH, quarry.pos.a);
    }

    // Wall areas lie along their facing direction.
    for (const auto& zone : tableState.wallZone) {
        if (zone.nb <= 0) continue;
        addObstacle(zone.pos, BLOCK_LENGTH, BLOCK_WIDTH, zone.pos.a);
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

    planner.input.start = Vec{start.x, start.y};
    planner.input.goal = Vec{goal.x, goal.y};

    // Plan for the heading the robot is steering towards, not the one it
    // happens to have right now. The drive controller turns towards goal.a
    // while it follows the path, so planning from the live heading made the
    // path a function of that same angle: a few degrees of rotation moved the
    // inflation, the path jumped to the other side of an obstacle, and the
    // robot oscillated left and right. Planning with the goal heading is the
    // one the controller is converging on, so the two no longer fight.
    planner.input.robot = nav::rotate_polygon(buildRobotPolygon(), goal.a);
    // inflate_polygon, not grow_polygon: the arc-sampled offset would turn the
    // 3-vertex hull into ~33 vertices and triple the planner's edge tests, and
    // rounding the corners is the opposite of what an asymmetric polygon is for.
    planner.input.robot = nav::inflate_polygon(planner.input.robot, ROBOT_CLEARANCE);

    planner.last = nav::plan_path(planner.input);
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
    // Remember the table so pathfind() can rebuild from it every call.
    planner.table = &tableState;
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
    if (planner.opponentIndex >= 0) {
        moveOpponent(center);
    } else {
        addOpponent(center);
    }
}

void pathfindClear() {
    resetInput();
    // The corner protection zones belong to the table, not to the match, so
    // they are part of "an empty table".
    addCornerZones();
    planner.last = PlanOutput{};
}

void pathfindAddObstacle(position_t center, double length, double width, double angleDeg) {
    addObstacle(center, length, width, angleDeg);
}

void pathfindToJson(json& out) {
    out = json::object();

    // Raw obstacles, as physical objects on the table. `kind` tells the
    // opponent apart from the stones for any consumer that wants to style them.
    json obstacles = json::array();
    for (int i = 0; i < planner.input.obstacle_count; i++) {
        const ObstacleInput& ob = planner.input.obstacles[i];
        const bool isOpponent = (i == planner.opponentIndex);
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
    std::array<Polygon, nav::kMaxShapes> shapes{};
    const int shapeCount = nav::build_config_shapes(planner.input, shapes);
    const int borderCount = planner.input.use_border ? 4 : 0;
    json inflated = json::array();
    for (int i = 0; i < shapeCount; i++) {
        // Shape order matches the obstacle order after the border block. The
        // `>= 0` guard matters: for border shapes the index is negative, and
        // with no opponent (-1) a missing guard flagged a wall as the opponent.
        const int obstacleIndex = i - borderCount;
        const bool isOpponent =
            (planner.opponentIndex >= 0 && obstacleIndex == planner.opponentIndex);
        inflated.push_back({{"corners", polygonToJson(shapes[i])},
                            {"border", i < borderCount},
                            {"opponent", isOpponent}});
    }
    out["inflated"] = inflated;

    json path = json::array();
    for (int i = 0; i < planner.last.point_count; i++) {
        path.push_back({{"x", planner.last.points[i].x}, {"y", planner.last.points[i].y}});
    }
    out["path"] = path;
    out["found"] = planner.last.found;
    out["length_mm"] = planner.last.found ? planner.last.length : 0.0;
}
