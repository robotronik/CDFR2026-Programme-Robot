#pragma once

#include "defs/structs.hpp"
#include "utils/json.hpp"

class TableState;
class Lidar;

/// Upper bound on the points a planned path can contain.
constexpr int PATHFIND_MAX_POINTS = 256;

/// Plans a path around the stones, wall areas and opponent.
///
/// @param path           buffer of at least PATHFIND_MAX_POINTS entries
/// @param path_length_mm polyline length in millimetres
/// @return number of points written, or 0 when the goal is unreachable
int pathfind(position_t start, position_t goal, position_t path[], double& path_length_mm);

/// Remembers the table the obstacles are rebuilt from on every pathfind().
void pathfindSetup(const TableState& tableState);

/// Updates the opponent position; it survives the per-call rebuild.
void pathfindUpdateOpponent(Lidar* lidar, const TableState& tableState);

/// Obstacles, inflated obstacles and the last path, for the web display.
void pathfindToJson(json& out);
