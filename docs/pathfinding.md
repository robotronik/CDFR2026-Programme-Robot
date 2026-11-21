# Pathfinding

The planner works in continuous space, not on a grid. The robot is a convex
polygon (a 400 mm equilateral triangle), stones are rotated rectangles and the
opponent is a 400 x 400 mm octagon, so the robot is never approximated by a
circle.

## How it works

1. **Configuration obstacles.** For each obstacle `O`, `C_obs = O (+) (-robot)`
   is the set of robot positions that collide with it.

   `C_obs` depends on heading, which is deliberate: the hull is rotated to
   `goal.a` before every plan, so the inflation follows the real silhouette.
   Planning at `goal.a` rather than the live heading matters, because the
   controller turns towards `goal.a` while it drives; planning from the live
   heading makes the path depend on the angle being changed, and the robot
   oscillates.

   The hull is grown by 5 mm so the controller has room to correct against a
   hairline contact. `inflate_polygon` keeps the triangle at 3 vertices;
   `grow_polygon` would arc-sample it to ~33 and round off the tight corners.

   A start inside a `C_obs` is moved out to the nearest free, drivable point.
   The goal is never moved: a goal inside a `C_obs` cannot be reached, so the
   plan fails, `pathfind()` returns 0 and logs why.

2. **Routing nodes.** Paths only bend at corners of the union of those polygons,
   so the nodes are the union boundary corners plus start and goal.

3. **Search.** A* over the visibility graph: an edge exists when the segment
   between two nodes stays outside every `C_obs`.

The table border is four walls inflated by the robot, and the four corners carry
protected zones (300 mm along x, 200 mm along y) the robot may never touch.

## Known limitation

The inflation uses the hull at one heading, `goal.a`, and assumes that describes
the robot for the whole move. The robot can strafe, so a segment is only safe if
the hull clears obstacles at every travel direction, which is not checked.
Making orientation part of the search state (`x, y, theta`) would fix this at
roughly 8x the nodes.

## Code layout

| File | Contents |
|---|---|
| `navigation/geom.hpp/.cpp` | Vectors, convex polygons, distances |
| `navigation/convexPathfind.hpp/.cpp` | `C_obs`, nodes, A* |
| `navigation/pathfind.hpp/.cpp` | API used by the rest of the robot |

```cpp
pathfindSetup(tableStatus);                         // start-up: remembers the table
pathfindUpdateOpponent(&lidar, tableStatus);        // every loop, cheap
int n = pathfind(start, goal, path, length_mm);     // 0 means no path
```

`pathfind()` rebuilds the stones and wall areas on every call, so a picked-up
stone simply stops being an obstacle on the next plan.

## Web display

`GET /get_global` and `GET /get_pathfinding` return:

```json
{
  "obstacles": [{ "kind": "stone", "corners": [{"x": -160, "y": 145}] }],
  "robot": [{"x": 230.9, "y": 0}, {"x": -115.5, "y": 200}, {"x": -115.5, "y": -200}],
  "inflated": [{ "border": true, "opponent": false, "corners": [] }],
  "path": [{"x": 0, "y": 1200}],
  "found": true,
  "length_mm": 2790.0
}
```

- `robot` is unrotated; the frontend translates it to the current position and
  applies the heading.
- `inflated` is what the planner actually avoids. Border walls come first with
  `"border": true`; the opponent carries `"opponent": true`.

## Testing locally

A desktop build starts in test mode (`TEST_API_ONLY` is defined when not
cross-compiling):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/programCDFR
```

Every 500 ms the table is re-randomized and a new path is planned. The random
layout goes through the table state, because `pathfind()` rebuilds from it.
