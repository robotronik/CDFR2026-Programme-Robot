#include "navigation/convexPathfind.hpp"
#include "utils/logger.hpp"

using namespace planner;

namespace {

// One square obstacle at (300, 0); the robot hull is 400 mm wide, so its
// configuration obstacle extends ~235 mm past the square's own edges.
PlanInput scene(Vec start, Vec goal) {
    PlanInput in;
    const Vec hull[] = {{230.94, 0}, {-115.47, 200}, {-115.47, -200}};
    in.robot = inflate_polygon(Polygon::from_points(hull, 3), 5.0);
    in.start = start;
    in.goal = goal;
    in.use_border = true;
    in.table_half_x = 1000.0;
    in.table_half_y = 1500.0;
    in.obstacle_count = 1;
    in.obstacles[0].kind = ObstacleKind::Poly;
    in.obstacles[0].poly = Polygon::rectangle(300, 0, 300, 300, 0.0);
    return in;
}

bool near(Vec a, Vec b) { return dist(a, b) < 1e-6; }

} // namespace

bool test_pathfind() {
    bool ok = true;

    // A goal inside a configuration obstacle must fail. It used to be relocated
    // to the nearest free point, so the search "succeeded" somewhere else: the
    // robot stopped short while the move still reported Done.
    {
        PlanOutput r = plan_path(scene({-600, -600}, {300, 0}));
        if (r.found || r.point_count != 0) {
            LOG_ERROR("Pathfind test: blocked goal produced a path");
            ok = false;
        }
        if (!r.goal_blocked) {
            LOG_ERROR("Pathfind test: blocked goal not flagged");
            ok = false;
        }
    }

    // A reachable goal must produce a path that starts at the start and ends
    // exactly on the requested goal.
    {
        const Vec start{-600, -600};
        const Vec goal{-500, 400};
        PlanOutput r = plan_path(scene(start, goal));
        if (!r.found || r.point_count < 2) {
            LOG_ERROR("Pathfind test: no path to a reachable goal");
            ok = false;
        } else {
            if (!near(r.points[0], start)) {
                LOG_ERROR("Pathfind test: path does not start at the start");
                ok = false;
            }
            if (!near(r.points[r.point_count - 1], goal)) {
                LOG_ERROR("Pathfind test: path does not end on the goal");
                ok = false;
            }
        }
    }

    // A start inside an obstacle is still escaped, and the path still lands on
    // the requested goal.
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
