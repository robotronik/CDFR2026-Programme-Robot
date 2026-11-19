#include "navigation/geom.hpp"

#include <algorithm>

namespace planner {
namespace {

// Wrap an angle difference into (-pi, pi]. C++ fmod keeps the sign of its
// argument, unlike Python's %, so the Python expression
// (b - a + pi) % (2 pi) - pi cannot be transliterated directly.
double wrap_angle(double d) {
    d = std::fmod(d + M_PI, 2.0 * M_PI);
    if (d < 0.0) d += 2.0 * M_PI;
    return d - M_PI;
}

// Convex hull of up to kMaxPolyVerts points (Andrew monotone chain), CCW.
Polygon hull_of(std::array<Vec, kMaxPolyVerts * kMaxPolyVerts>& buf, int count) {
    std::sort(buf.begin(), buf.begin() + count,
              [](const Vec& a, const Vec& b) {
                  return a.x != b.x ? a.x < b.x : a.y < b.y;
              });
    // Drop duplicates.
    int m = 0;
    for (int i = 0; i < count; ++i) {
        if (m > 0 && dist(buf[i], buf[m - 1]) < 1e-9) continue;
        buf[m++] = buf[i];
    }
    if (m <= 2) {
        Polygon p;
        for (int i = 0; i < m; ++i)
            p.pts[p.n++] = buf[i];
        p.finalize();
        return p;
    }

    auto cross = [](Vec o, Vec a, Vec b) { return (a - o).cross(b - o); };
    std::array<Vec, kMaxPolyVerts * kMaxPolyVerts> lower{}, upper{};
    int ln = 0, un = 0;
    for (int i = 0; i < m; ++i) {
        while (ln >= 2 && cross(lower[ln - 2], lower[ln - 1], buf[i]) <= 0)
            --ln;
        lower[ln++] = buf[i];
    }
    for (int i = m - 1; i >= 0; --i) {
        while (un >= 2 && cross(upper[un - 2], upper[un - 1], buf[i]) <= 0)
            --un;
        upper[un++] = buf[i];
    }

    Polygon p;
    for (int i = 0; i < ln - 1 && p.n < kMaxPolyVerts; ++i)
        p.pts[p.n++] = lower[i];
    for (int i = 0; i < un - 1 && p.n < kMaxPolyVerts; ++i)
        p.pts[p.n++] = upper[i];
    p.finalize();
    return p;
}

double point_segment_distance(Vec p, Vec a, Vec b) {
    const Vec ab = b - a;
    const double den = ab.dot(ab);
    const double t = den < kEps ? 0.0 : std::clamp((p - a).dot(ab) / den, 0.0, 1.0);
    return dist(p, a + ab * t);
}

} // namespace

void Polygon::finalize() {
    // Drop consecutive duplicates (zero-length edges have no usable normal).
    int m = 0;
    for (int i = 0; i < n; ++i) {
        if (m > 0 && dist(pts[i], pts[m - 1]) <= 1e-7) continue;
        pts[m++] = pts[i];
    }
    if (m > 2 && dist(pts[0], pts[m - 1]) <= 1e-7) --m;
    n = m;
    if (n < 3) return;

    double area = 0.0;
    for (int i = 0; i < n; ++i)
        area += pts[i].cross(pts[(i + 1) % n]);
    if (area < 0.0) std::reverse(pts.begin(), pts.begin() + n);

    for (int i = 0; i < n; ++i) {
        const Vec e = pts[(i + 1) % n] - pts[i];
        normals[i] = Vec{e.y, -e.x}.unit();
        double m_proj = -kInf;
        for (int k = 0; k < n; ++k)
            m_proj = std::max(m_proj, pts[k].dot(normals[i]));
        planes[i] = m_proj;
        plane_data[3 * i + 0] = normals[i].x;
        plane_data[3 * i + 1] = normals[i].y;
        plane_data[3 * i + 2] = m_proj;
    }

    min_corner = max_corner = pts[0];
    for (int i = 1; i < n; ++i) {
        min_corner.x = std::min(min_corner.x, pts[i].x);
        min_corner.y = std::min(min_corner.y, pts[i].y);
        max_corner.x = std::max(max_corner.x, pts[i].x);
        max_corner.y = std::max(max_corner.y, pts[i].y);
    }
}

Polygon Polygon::from_points(const Vec* v, int count) {
    std::array<Vec, kMaxPolyVerts * kMaxPolyVerts> buf{};
    for (int i = 0; i < count && i < int(buf.size()); ++i)
        buf[i] = v[i];
    return hull_of(buf, count);
}

Polygon Polygon::rectangle(double cx, double cy, double w, double h, double deg) {
    const Vec c{cx, cy};
    Vec v[4] = {
        c + rotate({-w / 2, -h / 2}, deg),
        c + rotate({w / 2, -h / 2}, deg),
        c + rotate({w / 2, h / 2}, deg),
        c + rotate({-w / 2, h / 2}, deg),
    };
    Polygon p;
    for (auto& q : v)
        p.pts[p.n++] = q;
    p.finalize();
    return p;
}

bool point_in_polygon(Vec p, const Polygon& poly, double tol) {
    for (int i = 0; i < poly.n; ++i) {
        if (p.dot(poly.normals[i]) > poly.planes[i] + tol) return false;
    }
    return true;
}

bool segment_enters_polygon(Vec a, Vec b, const Polygon& poly) {
    if (poly.n < 3) return false;

    // AABB rejection first: most candidate segments never come near.
    if (std::max(a.x, b.x) < poly.min_corner.x - kEps ||
        std::min(a.x, b.x) > poly.max_corner.x + kEps ||
        std::max(a.y, b.y) < poly.min_corner.y - kEps ||
        std::min(a.y, b.y) > poly.max_corner.y + kEps) {
        return false;
    }

    // Separating plane: if both endpoints are strictly outside one face there
    // is no crossing. This is the hot loop of the whole planner, so it reads
    // the interleaved plane array directly.
    const double* pd = poly.plane_data.data();
    for (int i = 0, lim = poly.n * 3; i < lim; i += 3) {
        const double nx = pd[i], ny = pd[i + 1], m = pd[i + 2];
        const double va = a.x * nx + a.y * ny;
        const double vb = b.x * nx + b.y * ny;
        if ((va < vb ? va : vb) > m) return false;
    }

    if (point_in_polygon(a, poly, -1e-9) || point_in_polygon(b, poly, -1e-9)) return true;

    // Clip the segment against every half-plane.
    const Vec d = b - a;
    double t0 = 0.0, t1 = 1.0;
    for (int i = 0; i < poly.n; ++i) {
        const Vec& nrm = poly.normals[i];
        const double denom = d.dot(nrm);
        const double off = poly.planes[i] - a.dot(nrm);
        if (std::abs(denom) < 1e-12) {
            if (off < -1e-9) return false;
            if (std::abs(off) <= 1e-9) return false; // sliding on this face
            continue;
        }
        const double t = off / denom;
        if (denom > 0.0)
            t1 = std::min(t1, t);
        else
            t0 = std::max(t0, t);
        if (t0 >= t1) return false;
    }
    return t1 - t0 > 1e-9;
}

double distance_to_polygon(Vec p, const Polygon& poly) {
    if (point_in_polygon(p, poly)) return 0.0;
    double best = kInf;
    for (int i = 0; i < poly.n; ++i) {
        best = std::min(best, point_segment_distance(p, poly.pts[i], poly.pts[(i + 1) % poly.n]));
    }
    return best;
}

double sat_gap(const Polygon& p, const Polygon& q) {
    bool separated = false;
    double best_sep = -kInf;
    double best_overlap = kInf;

    for (int side = 0; side < 2; ++side) {
        const Polygon& s = side == 0 ? p : q;
        for (int i = 0; i < s.n; ++i) {
            const Vec& nrm = s.normals[i];
            double min_p = kInf, max_p = -kInf, min_q = kInf, max_q = -kInf;
            for (int k = 0; k < p.n; ++k) {
                const double v = p.pts[k].dot(nrm);
                min_p = std::min(min_p, v);
                max_p = std::max(max_p, v);
            }
            for (int k = 0; k < q.n; ++k) {
                const double v = q.pts[k].dot(nrm);
                min_q = std::min(min_q, v);
                max_q = std::max(max_q, v);
            }
            const double gap = std::max(min_q - max_p, min_p - max_q);
            if (gap > 0.0) {
                separated = true;
                best_sep = std::max(best_sep, gap);
            } else {
                best_overlap = std::min(best_overlap, -gap);
            }
        }
    }
    if (!separated) return best_overlap == kInf ? 0.0 : -best_overlap;

    // Separated: SAT only gives a lower bound (closest points can be
    // vertex-to-vertex), so compute the true distance.
    double best = kInf;
    for (int i = 0; i < p.n; ++i) {
        for (int j = 0; j < q.n; ++j) {
            best = std::min(best, point_segment_distance(p.pts[i], q.pts[j], q.pts[(j + 1) % q.n]));
            best = std::min(best, point_segment_distance(q.pts[j], p.pts[i], p.pts[(i + 1) % p.n]));
        }
    }
    return best;
}

double circle_polygon_gap(Vec center, double radius, const Polygon& poly) {
    if (point_in_polygon(center, poly)) {
        double closest = kInf;
        for (int i = 0; i < poly.n; ++i) {
            closest = std::min(closest, point_segment_distance(center, poly.pts[i], poly.pts[(i + 1) % poly.n]));
        }
        return -(radius + closest);
    }
    double closest = kInf;
    for (int i = 0; i < poly.n; ++i) {
        closest = std::min(closest, point_segment_distance(center, poly.pts[i], poly.pts[(i + 1) % poly.n]));
    }
    return closest - radius;
}

double robot_rotation_radius(const Polygon& robot) {
    double best = 0.0;
    for (int i = 0; i < robot.n; ++i) {
        best = std::max(best, robot.pts[i].norm());
    }
    return best;
}

Polygon config_obstacle(const Polygon& obstacle, const Polygon& robot) {
    // C_obs = obstacle (+) (-robot): hull of all pairwise vertex sums.
    //
    // The robot is taken at its current heading, so the inflation follows the
    // real silhouette and the planner can use gaps a circle could not.
    std::array<Vec, kMaxPolyVerts * kMaxPolyVerts> buf{};
    int count = 0;
    for (int i = 0; i < obstacle.n; ++i) {
        for (int j = 0; j < robot.n; ++j) {
            buf[count++] = obstacle.pts[i] - robot.pts[j];
        }
    }
    return hull_of(buf, count);
}

Polygon config_circle(Vec center, double radius, const Polygon& robot) {
    // Exact boundary is center - v + R*u over robot vertices v and unit u.
    // Sample each corner arc, pushing samples out by the chord sagitta so the
    // polygon circumscribes the smooth offset (never dips inside it).
    constexpr int kArcSegments = 12;
    const double step = M_PI / (2 * kArcSegments);
    const double scale = 1.0 / std::cos(step);

    // Reflecting the robot flips its normals, so build (-robot) explicitly:
    // its outward normals point the other way and the corner arcs must follow
    // them, otherwise the sampled offset bulges inwards.
    // Mirror Python: reflect the vertices in place (which reverses the
    // winding) and let finalize() re-orient CCW and recompute the normals.
    Polygon neg;
    for (int i = 0; i < robot.n; ++i) {
        neg.pts[neg.n++] = Vec{robot.pts[i].x == 0.0 ? 0.0 : -robot.pts[i].x,
                               robot.pts[i].y == 0.0 ? 0.0 : -robot.pts[i].y};
    }
    neg.finalize();
    // Python's Polygon keeps the reflected order (already CCW); finalize()
    // must not have flipped it, otherwise the corner arcs sweep the wrong way.

    std::array<Vec, kMaxPolyVerts * kMaxPolyVerts> buf{};
    int count = 0;
    for (int i = 0; i < neg.n; ++i) {
        const Vec a = neg.pts[i];
        const Vec b = neg.pts[(i + 1) % neg.n];
        const Vec na = neg.normals[i];
        const Vec nb = neg.normals[(i + 1) % neg.n];
        buf[count++] = center + a + na * radius;
        buf[count++] = center + b + na * radius;

        const double ang_a = std::atan2(na.y, na.x);
        const double ang_b = std::atan2(nb.y, nb.x);
        const double d = wrap_angle(ang_b - ang_a);
        const int steps = std::max(1, int(std::abs(d) / step));
        for (int j = 1; j < steps; ++j) {
            const double ang = ang_a + d * (double(j) / steps);
            buf[count++] = center + b + Vec{std::cos(ang), std::sin(ang)} * (radius * scale);
        }
    }
    Polygon p;
    for (int i = 0; i < count && p.n < kMaxPolyVerts; ++i)
        p.pts[p.n++] = buf[i];
    p.finalize();
    return p;
}

Polygon grow_polygon(const Polygon& poly, double margin) {
    // Offset a convex polygon by `margin`: Minkowski sum with a disc, built as
    // "each edge pushed out, each corner bridged by a short arc".
    //
    // This mirrors the verified Python reference one-for-one: a 16-gon disc is
    // circumscribed by r = margin / cos(pi/16), and every edge emits both of
    // its endpoints (duplicates are merged later by finalize()). The result is
    // marginally conservative, which is what a safety margin wants.
    constexpr int kDiscSides = 16;
    const double step = M_PI / kDiscSides;
    const double r = margin / std::cos(step);

    Polygon out;
    for (int i = 0; i < poly.n && out.n + 2 * kDiscSides + 4 < kMaxPolyVerts; ++i) {
        const Vec a = poly.pts[i];
        const Vec b = poly.pts[(i + 1) % poly.n];
        const Vec na = poly.normals[i];
        const Vec nb = poly.normals[(i + 1) % poly.n];

        out.pts[out.n++] = a + na * r;
        out.pts[out.n++] = b + na * r;

        const double ang_a = std::atan2(na.y, na.x);
        const double ang_b = std::atan2(nb.y, nb.x);
        const double d = wrap_angle(ang_b - ang_a);
        const int steps = std::max(1, int(std::abs(d) / step));
        for (int j = 1; j < steps; ++j) {
            const double ang = ang_a + d * (double(j) / steps);
            out.pts[out.n++] = b + Vec{std::cos(ang), std::sin(ang)} * r;
        }
    }
    out.finalize();
    return out;
}

Polygon rotate_polygon(const Polygon& poly, double deg) {
    Polygon out;
    for (int i = 0; i < poly.n; ++i) {
        out.pts[out.n++] = rotate(poly.pts[i], deg);
    }
    out.finalize();
    return out;
}

Polygon inflate_polygon(const Polygon& poly, double margin) {
    Polygon out;
    for (int i = 0; i < poly.n; ++i) {
        const Vec v = poly.pts[i];
        const double len = v.norm();
        out.pts[out.n++] = len < kEps ? v : v + v * (margin / len);
    }
    out.finalize();
    return out;
}

} // namespace planner
