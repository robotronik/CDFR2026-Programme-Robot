// Convex geometry for the pathfinder, in millimetres in the table frame.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

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

// Enough for every configuration obstacle: 4-vertex stones and the triangle
// hull keep well under this, arc-sampled offsets are capped by it.
inline constexpr int kMaxPolyVerts = 160;
// 25 obstacles on the table plus the 4 border walls.
inline constexpr int kMaxShapes = 32;
inline constexpr int kMaxNodes = 768;
inline constexpr int kMaxPathPoints = 256;

struct Polygon {
        std::array<Vec, kMaxPolyVerts> pts{};
        std::array<Vec, kMaxPolyVerts> normals{};   // outward unit normal per edge
        std::array<double, kMaxPolyVerts> planes{}; // max projection along normals
        // `normals` and `planes` interleaved, so the collision loop walks one
        // contiguous array.
        std::array<double, 3 * kMaxPolyVerts> plane_data{};
        int n = 0;

        Vec min_corner{0, 0};
        Vec max_corner{0, 0};

        static Polygon from_points(const Vec* v, int count);
        static Polygon rectangle(double cx, double cy, double w, double h, double deg = 0.0);

        // Drops repeated vertices and orients CCW.
        void finalize();
};

// Signed distance: > 0 separated, < 0 penetrating.
double sat_gap(const Polygon& a, const Polygon& b);

bool segment_enters_polygon(Vec a, Vec b, const Polygon& poly);

// Distance from a point to a convex polygon, 0 if inside.
double distance_to_polygon(Vec p, const Polygon& poly);

bool point_in_polygon(Vec p, const Polygon& poly, double tol = 1e-9);

// C_obs = obstacle (+) (-robot), with the robot at its current heading.
Polygon config_obstacle(const Polygon& obstacle, const Polygon& robot);

// C_obs for a circular obstacle, as a circumscribing polygon.
Polygon config_circle(Vec center, double radius, const Polygon& robot);

// Minkowski sum with a disc, circumscribed so the result never dips below `margin`.
Polygon grow_polygon(const Polygon& poly, double margin);

Polygon rotate_polygon(const Polygon& poly, double deg);

// Pushes each vertex out along its own direction, keeping the vertex count.
Polygon inflate_polygon(const Polygon& poly, double margin);

} // namespace planner
