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

constexpr double OPPONENT_SIZE = 400.0;

constexpr double BLOCK_LENGTH = 320.0;
constexpr double BLOCK_WIDTH = 110.0;

constexpr double CORNER_SIZE_X = 300.0;
constexpr double CORNER_SIZE_Y = 200.0;

// Keeps the path off a hairline contact so the controller has room to correct.
constexpr double ROBOT_CLEARANCE = 5.0;

// The robot hull: a 400 mm equilateral triangle, one vertex forward (+x).
// TODO read the real dimensions from a single source of truth.
Polygon buildRobotPolygon() {
    constexpr double kSide = 400.0;
    constexpr double kR = kSide / 1.7320508075688772; // circumradius, s / sqrt(3)
    const Vec hull[] = {
        {kR, 0.0},
        {-kR / 2.0, kSide / 2.0},
        {-kR / 2.0, -kSide / 2.0},
    };
    return Polygon::from_points(hull, 3);
}

struct PlannerState {
        PlanInput input;
        PlanOutput last;
        const TableState* table = nullptr;
        // Slot of the opponent in `input.obstacles`, or -1.
        int opponentIndex = -1;
};

PlannerState state;

void rebuildObstacles();

void resetInput() {
    state.input = PlanInput{};
    state.opponentIndex = -1;
    state.input.robot = buildRobotPolygon();
    state.input.use_border = true;
    state.input.table_half_x = 1000.0;
    state.input.table_half_y = 1500.0;
    state.input.margin = 0.0;
}

// `x_extent` and `y_extent` are full side lengths, before rotation.
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
    constexpr double kCut = kHalf * 0.41421356237309503; // kHalf * tan(22.5 deg)
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
    // rebuildObstacles() reads this back to carry the opponent over.
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

// Moves the last obstacle into the opponent's slot to keep the array contiguous.
void removeOpponent() {
    if (state.opponentIndex < 0) return;
    const int last = state.input.obstacle_count - 1;
    if (state.opponentIndex != last) {
        state.input.obstacles[state.opponentIndex] = state.input.obstacles[last];
    }
    state.input.obstacle_count = last;
    state.opponentIndex = -1;
}

// One protected rectangle per table corner, flush with it.
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

    for (const auto& quarry : tableState.quarry) {
        if (quarry.nb <= 0) continue;
        addObstacle(quarry.pos, BLOCK_WIDTH, BLOCK_LENGTH, quarry.pos.a);
    }

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

    // Rebuilt every call: stones are picked up and dropped, so a cache goes
    // stale as soon as a quarry's count changes.
    rebuildObstacles();

    state.input.start = Vec{start.x, start.y};
    state.input.goal = Vec{goal.x, goal.y};

    // Plan at the target heading: the controller turns towards goal.a as it
    // drives, and planning from the live heading made the path depend on the
    // very angle being changed, which made the robot oscillate.
    state.input.robot = planner::rotate_polygon(buildRobotPolygon(), goal.a);
    // inflate_polygon, not grow_polygon: the arc-sampled offset would turn the
    // 3-vertex hull into ~33 vertices.
    state.input.robot = planner::inflate_polygon(state.input.robot, ROBOT_CLEARANCE);
    state.last = planner::plan_path(state.input);

    if (!state.last.found || state.last.point_count <= 0) {
        if (state.last.goal_blocked) {
            LOG_ERROR("Pathfinding: target is inside an obstacle, not moving");
        } else {
            LOG_WARNING("Pathfinding: no path to the target");
        }
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
    state.table = &tableState;
    rebuildObstacles();
}

void pathfindUpdateOpponent(Lidar* lidar, const TableState& tableState) {
    if (lidar == nullptr) return;

    // The lidar pipeline already tracks the opponent.
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

    json obstacles = json::array();
    for (int i = 0; i < state.input.obstacle_count; i++) {
        const ObstacleInput& ob = state.input.obstacles[i];
        const bool isOpponent = (i == state.opponentIndex);
        obstacles.push_back({{"kind", isOpponent ? "opponent" : "stone"},
                             {"corners", polygonToJson(ob.poly)}});
    }
    out["obstacles"] = obstacles;

    // Unrotated: the frontend places it at `pos` and applies the heading itself.
    out["robot"] = polygonToJson(buildRobotPolygon());

    // C_obs per obstacle, border walls first, so the frontend can tint them.
    std::array<Polygon, planner::kMaxShapes> shapes{};
    const int shapeCount = planner::build_config_shapes(state.input, shapes);
    const int borderCount = state.input.use_border ? 4 : 0;
    json inflated = json::array();
    for (int i = 0; i < shapeCount; i++) {
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
