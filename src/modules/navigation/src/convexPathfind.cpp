#include "navigation/convexPathfind.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace planner {
namespace {

// Closest distance between two segments; t/u locate it on each segment.
struct SegPair {
        double d;
        double t;
        double u;
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

inline bool aabb_overlap(Vec a, Vec b, const Polygon& poly) {
    return !(std::max(a.x, b.x) < poly.min_corner.x ||
             std::min(a.x, b.x) > poly.max_corner.x ||
             std::max(a.y, b.y) < poly.min_corner.y ||
             std::min(a.y, b.y) > poly.max_corner.y);
}

bool segment_blocked(Vec a, Vec b, const std::array<Polygon, kMaxShapes>& shapes, int count) {
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

// Routing nodes on the union boundary: visible vertices, edge crossings and
// closest-approach points (the last bound narrow passages).
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

// Drop waypoints whose removal leaves a clear straight segment.
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

    // Border walls are inflated like any other obstacle.
    if (input.use_border) {
        constexpr double kWall = 50.0;
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

    // A point inside a configuration obstacle cannot start a path, since every
    // segment leaving it reads as blocked. Walk it out along the nearest edge
    // normal that leads somewhere free and drivable; the boundary itself is
    // still "inside", so the walk steps past it. Corner zones sit flush with
    // the table, so their outward normal leads off the field: try every edge.
    constexpr double kEscapeStep = 1.0; // mm
    constexpr int kEscapeTries = 256;
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

    // Only the start is escaped. Moving the goal would let the search succeed
    // elsewhere and report Done at the wrong place; a blocked goal is simply
    // unreachable, which the search below reports as no path.
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

    // Lazy A*: each unordered pair is tested at most once.
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

    // One push per relaxation, so the heap must grow.
    struct HeapItem {
            double f;
            int node;
    };
    struct HeapCompare {
            bool operator()(const HeapItem& a, const HeapItem& b) const {
                return a.f != b.f ? a.f > b.f : a.node > b.node;
            }
    };
    std::priority_queue<HeapItem, std::vector<HeapItem>, HeapCompare> heap;
    auto heap_push = [&](double f, int node) { heap.push({f, node}); };

    // Pairs already tested; n <= kMaxNodes keeps this small.
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
        return !segment_blocked(nodes.pts[a], nodes.pts[b], shapes, shape_count);
    };

    // Each expansion scans the still-open nodes; the prune skips pairs that
    // cannot beat the incumbent.
    double best_complete = kInf;
    g[si] = 0.0;
    heap_push(dist(nodes.pts[si], nodes.pts[gi]), si);
    while (!heap.empty()) {
        const HeapItem top = heap.top();
        heap.pop();
        const int u = top.node;
        if (closed[u]) continue;
        closed[u] = 1;
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

    // Unreachable: the walk below would otherwise return a single point.
    if (g[gi] == kInf) return result;

    NodeBuffer raw;
    for (int c = gi; c != -1; c = parent[c]) {
        if (raw.n < kMaxNodes) raw.pts[raw.n++] = nodes.pts[c];
    }
    std::reverse(raw.pts.begin(), raw.pts.begin() + raw.n);

    NodeBuffer simplified;
    simplify(raw, shapes, shape_count, simplified);

    // The path must end on the requested goal, never short of it.
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
