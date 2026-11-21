#include "navigation/convexPathfind.hpp"
#include "utils/logger.hpp"

using namespace planner;

namespace {

// One 300 mm square at (300, 0); the robot hull is 400 mm wide.
PlanInput scene(Vec start, Vec goal) {
    PlanInput in;
    const Vec hull[] = {{230.94, 0}, {-115.47, 200}, {-115.47, -200}};
    in.robot = inflate_polygon(Polygon::from_points(hull, 3), 5.0);
    in.start = start;
    in.goal = goal;
    in.obstacle_count = 1;
    in.obstacles[0].poly = Polygon::rectangle(300, 0, 300, 300, 0.0);
    return in;
}

bool near(Vec a, Vec b) { return dist(a, b) < 1e-6; }

} // namespace

bool test_pathfind() {
    bool ok = true;

    // A goal inside a configuration obstacle must fail rather than be relocated.
    {
        PlanOutput r = plan_path(scene({-600, -600}, {300, 0}));
        if (r.found || r.point_count != 0 || !r.goal_blocked) {
            LOG_ERROR("Pathfind test: blocked goal produced a path");
            ok = false;
        }
    }

    // A reachable goal: the path starts at the start and ends on the goal.
    {
        const Vec start{-600, -600};
        const Vec goal{-500, 400};
        PlanOutput r = plan_path(scene(start, goal));
        if (!r.found || r.point_count < 2) {
            LOG_ERROR("Pathfind test: no path to a reachable goal");
            ok = false;
        } else if (!near(r.points[0], start) || !near(r.points[r.point_count - 1], goal)) {
            LOG_ERROR("Pathfind test: path does not run from start to goal");
            ok = false;
        }
    }

    // A start inside an obstacle is escaped, and the path still ends on the goal.
    {
        const Vec goal{-800, 0};
        PlanOutput r = plan_path(scene({300, 0}, goal));
        if (!r.found || r.point_count < 2) {
            LOG_ERROR("Pathfind test: escaped start produced no path");
            ok = false;
        } else if (!near(r.points[r.point_count - 1], goal)) {
            LOG_ERROR("Pathfind test: escaped-start path misses the goal");
            ok = false;
        }
    }

    if (ok) LOG_INFO("Pathfind test - all planning invariants hold");
    return ok;
}
