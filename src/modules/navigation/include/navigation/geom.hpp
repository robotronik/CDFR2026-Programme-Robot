// Convex geometry for the grid-free pathfinder (C++ port).
//
// Everything is millimetres in the table frame: x in [-1000, 1000],
// y in [-1500, 1500]. No allocation happens in the hot paths: polygons are
// fixed-capacity, and the collision tests are plain arithmetic.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace planner {

inline constexpr double kEps = 1e-9;
inline constexpr double kInf = std::numeric_limits<double>::infinity();

struct Vec {
        double x = 0.0;
        double y = 0.0;

        Vec() = default;
        Vec(double x_, double y_)
            : x(x_), y(y_) {}

        Vec operator+(Vec o) const { return {x + o.x, y + o.y}; }
        Vec operator-(Vec o) const { return {x - o.x, y - o.y}; }
        Vec operator*(double k) const { return {x * k, y * k}; }
        Vec operator/(double k) const { return {x / k, y / k}; }
        Vec& operator+=(Vec o) {
            x += o.x;
            y += o.y;
            return *this;
        }

        double dot(Vec o) const { return x * o.x + y * o.y; }
        double cross(Vec o) const { return x * o.y - y * o.x; }
        double norm() const { return std::sqrt(x * x + y * y); }
        Vec unit() const {
            const double n = norm();
            return n < kEps ? Vec{0.0, 0.0} : Vec{x / n, y / n};
        }
};

inline double dist(Vec a, Vec b) {
    return (a - b).norm();
}

inline Vec rotate(Vec v, double deg) {
    const double a = deg * M_PI / 180.0;
    const double c = std::cos(a), s = std::sin(a);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

// Maximum vertices any configuration obstacle can need. A stone is 4, the robot
// hull 5, so the Minkowski sum is <= 9; the circular offset is sampled, and we
// cap it explicitly. Everything is fixed-size so nothing allocates.
inline constexpr int kMaxPolyVerts = 160;
// Worst case on the table: 10 quarries + 10 wall areas + 4 corner protection
// zones + the opponent = 25 obstacles, plus the 4 border walls the planner adds
// itself. This used to be 16, which silently dropped every stone past the limit
// so paths ran straight through them.
inline constexpr int kMaxShapes = 32;
inline constexpr int kMaxNodes = 768;
inline constexpr int kMaxPathPoints = 256;

struct Polygon {
        std::array<Vec, kMaxPolyVerts> pts{};
        std::array<Vec, kMaxPolyVerts> normals{};   // outward unit normal per edge
        std::array<double, kMaxPolyVerts> planes{}; // max projection along normals
        // Same data as `normals`/`planes`, interleaved (nx, ny, m) so the collision
        // loop walks one contiguous array: fewer cache lines, no pointer chasing.
        std::array<double, 3 * kMaxPolyVerts> plane_data{};
        int n = 0;

        Vec min_corner{0, 0};
        Vec max_corner{0, 0};

        // Builds the polygon from unsorted/duplicate vertices, dropping zero-length
        // edges (their normals poison the half-plane tests) and orienting CCW.
        static Polygon from_points(const Vec* v, int count);
        static Polygon rectangle(double cx, double cy, double w, double h, double deg = 0.0);

        void finalize();

        Polygon translated(Vec t) const {
            Polygon r = *this;
            for (int i = 0; i < r.n; ++i)
                r.pts[i] += t;
            r.min_corner += t;
            r.max_corner += t;
            return r;
        }
};

// Signed distance between two convex polygons: > 0 separated, < 0 penetration.
double sat_gap(const Polygon& a, const Polygon& b);

// Signed distance between a circle and a convex polygon.
double circle_polygon_gap(Vec center, double radius, const Polygon& poly);

// True if the segment has an interior point strictly inside the polygon.
bool segment_enters_polygon(Vec a, Vec b, const Polygon& poly);

// Exact distance from a point to a convex polygon (0 if inside).
double distance_to_polygon(Vec p, const Polygon& poly);

// True if p lies in the interior (boundary excluded by `tol`).
bool point_in_polygon(Vec p, const Polygon& poly, double tol = 1e-9);

// Radius of the circle traced by the robot's farthest vertex when it spins
// about its reference point.
double robot_rotation_radius(const Polygon& robot);

// C_obs = obstacle (+) (-robot): positions of the robot reference point that
// collide with `obstacle`, with `robot` taken at its current heading. This is
// the tight, orientation-dependent inflation: it follows the real silhouette,
// so the planner can use gaps a circle could not. Callers must pass the robot
// already rotated to the live heading (add any clearance themselves).
Polygon config_obstacle(const Polygon& obstacle, const Polygon& robot);

// C_obs for a circular obstacle, as a circumscribing polygon.
Polygon config_circle(Vec center, double radius, const Polygon& robot);

// Offset a convex polygon outwards by `margin` (Minkowski sum with a disc,
// circumscribed so the result never dips below the margin).
Polygon grow_polygon(const Polygon& poly, double margin);

// Rotate a polygon about the origin by `deg` degrees (counter-clockwise).
Polygon rotate_polygon(const Polygon& poly, double deg);

// Grow a polygon by `margin` while keeping its vertex count: each vertex is
// pushed out along its own direction. Unlike grow_polygon this does not bridge
// corners with arcs, so the shape stays small and cheap. Intended for small
// safety margins where the arc detail would cost far more than it is worth.
Polygon inflate_polygon(const Polygon& poly, double margin);

} // namespace planner
