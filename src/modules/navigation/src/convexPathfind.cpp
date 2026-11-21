#include "navigation/convexPathfind.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace planner {
namespace {

// Distance between two segments, and a pair of points realising it.
struct SegPair {
        double d;
        double t; // parameter along the first segment
        double u; // parameter along the second segment
};

SegPair segment_segment(Vec a0, Vec a1, Vec b0, Vec b1) {
    const Vec da = a1 - a0;
    const Vec db = b1 - b0;
    const double den = da.cross(db);
    if (std::abs(den) > 1e-12) {
        const double t = (b0 - a0).cross(db) / den;
        const double u = (b0 - a0).cross(da) / den;
        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) return {0.0, t, u};
    }
    // Endpoint-to-segment minimum.
    SegPair best{kInf, 0.0, 0.0};
    auto consider = [&](Vec p, Vec s0, Vec s1, bool first) {
        const Vec sd = s1 - s0;
        const double dd = sd.dot(sd);
        const double k = dd < kEps ? 0.0 : std::clamp((p - s0).dot(sd) / dd, 0.0, 1.0);
        const double d = dist(p, s0 + sd * k);
        if (d < best.d) best = {d, first ? k : 0.0, first ? 0.0 : k};
    };
    consider(a0, b0, b1, true);
    consider(a1, b0, b1, true);
    consider(b0, a0, a1, false);
    consider(b1, a0, a1, false);
    return best;
}

// Cheap exact reject: the segment's AABB must overlap the shape's AABB.
inline bool aabb_overlap(Vec a, Vec b, const Polygon& poly) {
    return !(std::max(a.x, b.x) < poly.min_corner.x ||
             std::min(a.x, b.x) > poly.max_corner.x ||
             std::max(a.y, b.y) < poly.min_corner.y ||
             std::min(a.y, b.y) > poly.max_corner.y);
}

bool segment_blocked(Vec a, Vec b, const std::array<Polygon, kMaxShapes>& shapes, int count) {
    // AABB test first: a long segment usually only comes near one or two
    // shapes, so this skips the per-vertex work for the rest.
    for (int i = 0; i < count; ++i) {
        if (!aabb_overlap(a, b, shapes[i])) continue;
        if (segment_enters_polygon(a, b, shapes[i])) return true;
    }
    return false;
}

bool inside_any_strict(Vec p, const std::array<Polygon, kMaxShapes>& shapes, int count) {
    for (int i = 0; i < count; ++i) {
        if (point_in_polygon(p, shapes[i], -1e-6)) return true;
    }
    return false;
}

double distance_to_set(Vec p, const std::array<Polygon, kMaxShapes>& shapes, int count) {
    double best = kInf;
    for (int i = 0; i < count; ++i)
        best = std::min(best, distance_to_polygon(p, shapes[i]));
    return best;
}

struct NodeBuffer {
        std::array<Vec, kMaxNodes> pts{};
        int n = 0;

        bool add(Vec p) {
            if (n >= kMaxNodes) return false;
            for (int i = 0; i < n; ++i) {
                if (dist(pts[i], p) < 1e-6) return false;
            }
            pts[n++] = p;
            return true;
        }
};

// Union-boundary routing nodes: vertices not swallowed by another shape, plus
// edge crossings and closest-approach points (which bound narrow passages and
// otherwise leave a genuine gap looking disconnected).
void collect_union_nodes(const std::array<Polygon, kMaxShapes>& shapes, int count,
                         NodeBuffer& out) {
    for (int s = 0; s < count; ++s) {
        for (int v = 0; v < shapes[s].n; ++v) {
            const Vec p = shapes[s].pts[v];
            bool buried = false;
            for (int o = 0; o < count && !buried; ++o) {
                if (o != s && point_in_polygon(p, shapes[o], -1e-7)) buried = true;
            }
            if (!buried) out.add(p);
        }
    }

    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            const Polygon& A = shapes[i];
            const Polygon& B = shapes[j];

            // Edge crossings.
            for (int e = 0; e < A.n; ++e) {
                for (int f = 0; f < B.n; ++f) {
                    const SegPair sp = segment_segment(A.pts[e], A.pts[(e + 1) % A.n],
                                                       B.pts[f], B.pts[(f + 1) % B.n]);
                    if (sp.d > 1e-9) continue;
                    const Vec p = A.pts[e] + (A.pts[(e + 1) % A.n] - A.pts[e]) * sp.t;
                    bool buried = false;
                    for (int o = 0; o < count && !buried; ++o) {
                        if (point_in_polygon(p, shapes[o], -1e-7)) buried = true;
                    }
                    if (!buried) out.add(p);
                }
            }

            // Closest approach, only for genuinely separated shapes.
            if (sat_gap(A, B) <= 1e-6) continue;
            double best_d = kInf;
            Vec best_p{}, best_q{};
            for (int e = 0; e < A.n; ++e) {
                for (int f = 0; f < B.n; ++f) {
                    const SegPair sp = segment_segment(A.pts[e], A.pts[(e + 1) % A.n],
                                                       B.pts[f], B.pts[(f + 1) % B.n]);
                    if (sp.d < best_d - 1e-9) {
                        best_d = sp.d;
                        best_p = A.pts[e] + (A.pts[(e + 1) % A.n] - A.pts[e]) * sp.t;
                        best_q = B.pts[f] + (B.pts[(f + 1) % B.n] - B.pts[f]) * sp.u;
                    }
                }
            }
            if (best_d <= 1e-6 || best_d == kInf) continue;
            for (Vec p : {best_p, best_q}) {
                bool buried = false;
                for (int o = 0; o < count && !buried; ++o) {
                    if (point_in_polygon(p, shapes[o], -1e-6)) buried = true;
                }
                if (!buried) out.add(p);
            }
        }
    }
}

// Drop waypoints whose removal a straight replacement segment makes safe.
void simplify(NodeBuffer& path, const std::array<Polygon, kMaxShapes>& shapes, int count,
              NodeBuffer& out) {
    if (path.n <= 2) {
        out = path;
        return;
    }
    out.add(path.pts[0]);
    int i = 0;
    while (i < path.n - 1) {
        int j = path.n - 1;
        while (j > i + 1 && segment_blocked(path.pts[i], path.pts[j], shapes, count))
            --j;
        out.add(path.pts[j]);
        i = j;
    }
}

} // namespace

int build_config_shapes(const PlanInput& input, std::array<Polygon, kMaxShapes>& out) {
    int n = 0;

    // Table border: four thin walls around the field, treated exactly like
    // any other obstacle (inflated by the robot and by `margin`). This mirrors
    // the robot firmware's pathfind_place_border().
    if (input.use_border) {
        constexpr double kWall = 50.0; // wall thickness, as on the robot
        const double hx = input.table_half_x;
        const double hy = input.table_half_y;
        const Polygon walls[4] = {
            Polygon::rectangle(0.0, -hy, 2.0 * hx + 2.0 * kWall, kWall),
            Polygon::rectangle(0.0, hy, 2.0 * hx + 2.0 * kWall, kWall),
            Polygon::rectangle(-hx, 0.0, kWall, 2.0 * hy + 2.0 * kWall),
            Polygon::rectangle(hx, 0.0, kWall, 2.0 * hy + 2.0 * kWall),
        };
        for (const Polygon& w : walls) {
            if (n >= kMaxShapes) break;
            Polygon c = config_obstacle(w, input.robot);
            if (input.margin > 0.0) c = grow_polygon(c, input.margin);
            out[n++] = c;
        }
    }

    for (int i = 0; i < input.obstacle_count && n < kMaxShapes; ++i) {
        const ObstacleInput& ob = input.obstacles[i];
        Polygon c = ob.kind == ObstacleKind::Circle
                        ? config_circle(ob.center, ob.radius, input.robot)
                        : config_obstacle(ob.poly, input.robot);
        if (input.margin > 0.0) c = grow_polygon(c, input.margin);
        out[n++] = c;
    }
    return n;
}

PlanOutput plan_path(const PlanInput& input) {
    PlanOutput result;

    std::array<Polygon, kMaxShapes> shapes{};
    const int shape_count = build_config_shapes(input, shapes);

    // If `p` sits inside a configuration obstacle the robot is already
    // colliding there, and no path can leave it: every segment starting inside
    // a polygon reads as blocked. Push `p` out to the nearest free point so the
    // plan starts with the shortest route out of the obstacle.
    //
    // Two traps to avoid. The nearest point on the boundary lies *on* the edge,
    // so segment tests still see it as entering the polygon -- step past it.
    // And the nearest edge is not always escapable: the corner protection zones
    // are flush with the table, so their outward normal points off the field.
    // Try every edge and keep the closest exit that is actually outside all
    // shapes.
    constexpr double kEscapeStep = 1.0;  // mm past the boundary
    constexpr int kEscapeTries = 256;    // clearance steps per candidate
    auto escape_if_blocked = [&](Vec p) {
        if (distance_to_set(p, shapes, shape_count) > 1e-6) return p;

        Vec best = p;
        double best_d = kInf;
        for (int s = 0; s < shape_count; ++s) {
            for (int i = 0; i < shapes[s].n; ++i) {
                const Vec a = shapes[s].pts[i];
                const Vec b = shapes[s].pts[(i + 1) % shapes[s].n];
                const Vec ab = b - a;
                const double den = ab.dot(ab);
                const double t = den < 1e-9 ? 0.0 : std::clamp((p - a).dot(ab) / den, 0.0, 1.0);
                const Vec q = a + ab * t;
                const double d = dist(p, q);
                if (d >= best_d) continue;

                // Walk out along this edge's normal until the point is clear of
                // every shape. Borders and neighbouring zones can sit in the
                // way, so this may take several steps.
                const Vec n = shapes[s].normals[i];
                Vec out = q;
                bool escaped = false;
                for (int k = 0; k < kEscapeTries; ++k) {
                    out = out + n * kEscapeStep;
                    if (distance_to_set(out, shapes, shape_count) > 1e-6) {
                        escaped = true;
                        break;
                    }
                }
                // Prefer an exit the robot can actually occupy. The corner
                // zones sit flush with the table, so their nearest edge leads
                // straight off the field; a slightly longer exit inward is the
                // one that can be driven.
                if (!escaped) continue;
                const bool off_table = input.use_border &&
                                       (std::fabs(out.x) > input.table_half_x ||
                                        std::fabs(out.y) > input.table_half_y);
                if (off_table) continue;
                if (d < best_d) {
                    best_d = d;
                    best = out;
                }
            }
        }
        return best;
    };

    // Only the start is escaped. The goal is the caller's real target: moving it
    // would let the search "succeed" somewhere else and the drive controller
    // would then stop short while the move still reports Done. A goal inside a
    // configuration obstacle is genuinely unreachable (every segment ending
    // there enters it), so the search below just finds no path, which callers
    // already treat as a failure.
    const Vec start = escape_if_blocked(input.start);
    const Vec goal = input.goal;
    result.goal_blocked = distance_to_set(goal, shapes, shape_count) <= 1e-6;

    NodeBuffer candidates;
    candidates.add(start);
    candidates.add(goal);
    collect_union_nodes(shapes, shape_count, candidates);

    NodeBuffer nodes;
    for (int i = 0; i < candidates.n; ++i) {
        if (!inside_any_strict(candidates.pts[i], shapes, shape_count)) nodes.add(candidates.pts[i]);
    }
    nodes.add(start);
    nodes.add(goal);
    const int n = nodes.n;
    result.nodes = n;

    // Lazy A*: neighbours are generated on demand, and each unordered pair is
    // tested at most once.
    std::array<double, kMaxNodes> g{};
    std::array<int, kMaxNodes> parent{};
    std::array<uint8_t, kMaxNodes> closed{};
    for (int i = 0; i < n; ++i) {
        g[i] = kInf;
        parent[i] = -1;
        closed[i] = 0;
    }

    auto index_of = [&](Vec p) {
        for (int i = 0; i < n; ++i)
            if (dist(nodes.pts[i], p) < 1e-6) return i;
        return -1;
    };
    const int si = index_of(start);
    const int gi = index_of(goal);
    if (si < 0 || gi < 0) return result;

    // Min-heap over (f, node). One push per successful relaxation: a fixed-size
    // array here would write past its end once that count exceeds it, so the
    // heap owns its storage and grows as needed.
    struct HeapItem {
            double f;
            int node;
    };
    struct HeapCompare {
            bool operator()(const HeapItem& a, const HeapItem& b) const {
                // Ties on f go to the lower node index, the same order the
                // previous fixed heap produced.
                return a.f != b.f ? a.f > b.f : a.node > b.node;
            }
    };
    std::priority_queue<HeapItem, std::vector<HeapItem>, HeapCompare> heap;
    auto heap_push = [&](double f, int node) { heap.push({f, node}); };

    // Pair-tested bitmap: n <= kMaxNodes, so a flat bitset is fine.
    static thread_local std::array<uint64_t, (kMaxNodes * kMaxNodes + 63) / 64> tested_bits{};
    tested_bits.fill(0);
    auto pair_key = [&](int a, int b) -> size_t {
        if (a > b) std::swap(a, b);
        return size_t(a) * kMaxNodes + b;
    };
    auto pair_seen = [&](int a, int b) {
        const size_t k = pair_key(a, b);
        return (tested_bits[k >> 6] >> (k & 63)) & 1u;
    };
    auto pair_mark = [&](int a, int b) {
        const size_t k = pair_key(a, b);
        tested_bits[k >> 6] |= (uint64_t(1) << (k & 63));
    };
    auto visible = [&](int a, int b) {
        if (pair_seen(a, b)) return false;
        pair_mark(a, b);
        ++result.edge_tests;
        return !segment_blocked(nodes.pts[a], nodes.pts[b], shapes, shape_count);
    };

    // A* over the lazy visibility graph. Each expansion scans the still-open
    // nodes; every unordered pair is tested at most once (bitmap), and a sound
    // lower-bound prune skips pairs that cannot beat the incumbent.
    double best_complete = kInf;
    g[si] = 0.0;
    heap_push(dist(nodes.pts[si], nodes.pts[gi]), si);
    while (!heap.empty()) {
        const HeapItem top = heap.top();
        heap.pop();
        const int u = top.node;
        if (closed[u]) continue;
        closed[u] = 1;
        ++result.expanded;
        if (u == gi) {
            best_complete = g[gi];
            break;
        }
        const double h_u = dist(nodes.pts[u], nodes.pts[gi]);
        for (int v = 0; v < n; ++v) {
            if (v == u || closed[v]) continue;
            const double d_uv = dist(nodes.pts[u], nodes.pts[v]);
            if (g[u] + d_uv + std::abs(dist(nodes.pts[v], nodes.pts[gi]) - h_u) >= best_complete) {
                continue;
            }
            if (!visible(u, v)) continue;
            const double ng = g[u] + d_uv;
            if (ng < g[v] - 1e-9) {
                g[v] = ng;
                parent[v] = u;
                heap_push(ng + dist(nodes.pts[v], nodes.pts[gi]), v);
            }
        }
    }

    // Unreachable: A* never relaxed the goal, so g[gi] is still infinite.
    // Without this check the walk below would "succeed" with a single point.
    if (g[gi] == kInf) return result;

    NodeBuffer raw;
    for (int c = gi; c != -1; c = parent[c]) {
        // Build in reverse, then flip.
        if (raw.n < kMaxNodes) raw.pts[raw.n++] = nodes.pts[c];
    }
    std::reverse(raw.pts.begin(), raw.pts.begin() + raw.n);

    NodeBuffer simplified;
    simplify(raw, shapes, shape_count, simplified);

    // The path must end on the requested goal; stopping short would report Done
    // at the wrong place. The search reaches the goal as a node and simplify()
    // always keeps the last point, so this holds by construction -- keep the
    // check so a future change cannot silently reintroduce the drift.
    if (simplified.n == 0 || dist(simplified.pts[simplified.n - 1], goal) > 1e-6) {
        if (simplified.n < kMaxPathPoints) simplified.add(goal);
    }

    result.found = true;
    result.length = 0.0;
    for (int i = 0; i < simplified.n && i < kMaxPathPoints; ++i) {
        result.points[i] = simplified.pts[i];
        ++result.point_count;
        if (i > 0) result.length += dist(simplified.pts[i - 1], simplified.pts[i]);
    }
    return result;
}

} // namespace planner
