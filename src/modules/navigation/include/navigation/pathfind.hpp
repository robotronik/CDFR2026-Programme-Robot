#pragma once

#include "defs/structs.hpp"
#include "utils/json.hpp"

class TableState;
class Lidar;

/// Upper bound on the points a planned path can contain.
constexpr int PATHFIND_MAX_POINTS = 256;

/// Plans a collision-free path for the robot's convex hull.
///
/// The obstacles are the stones and wall areas held by `tableState`, plus the
/// opponent detected by the lidar. The configuration-space obstacles are exact
/// (obstacle (+) (-robot)), so the robot is never approximated by a circle.
///
/// @param start         first point of the returned path
/// @param goal          last point of the returned path
/// @param path          output buffer of at least PATHFIND_MAX_POINTS entries
/// @param path_length_mm polyline length in millimetres
/// @return number of points written, or 0 when no path exists
int pathfind(position_t start, position_t goal, position_t path[], double& path_length_mm);

/// Remembers the table the obstacles are read from. Call once at start-up.
/// pathfind() rebuilds the stones and wall areas from it on every call, so a
/// picked-up stone stops blocking the path without any extra bookkeeping.
void pathfindSetup(const TableState& tableState);

/// Updates the observed opponent position. Call whenever the lidar produces a
/// new estimate; the tracked position survives the per-call rebuild.
void pathfindUpdateOpponent(Lidar* lidar, const TableState& tableState);

/// Clears every obstacle, so the next setup starts from an empty table.
void pathfindClear();

/// Adds a rectangular stone at an arbitrary angle (table millimetres).
void pathfindAddObstacle(position_t center, double length, double width, double angle_deg);

/// Serializes the obstacles and the last planned path for the web display.
///
/// The frontend draws `obstacles` (rotated rectangles plus one circle for the
/// opponent) instead of the old grid costmap.
void pathfindToJson(json& out);
