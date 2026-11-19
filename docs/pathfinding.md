# Pathfinding

The robot plans on the exact table, not on a grid. The robot is a convex
polygon (an equilateral triangle with 400 mm sides), the stones are rotated
rectangles and the opponent is an octagon inside a 400 x 400 mm footprint, so a circle approximation is
never used for the robot.

## How it works

1. **Configuration obstacles.** For each obstacle `O` the planner computes

   ```
   C_obs = O (+) (-robot)
   ```

   the set of robot positions that collide with `O`. It is a convex polygon and
   does not depend on the direction of travel.

   It *does* depend on the robot's heading, and that is deliberate. The hull is
   rotated to the heading the robot is steering towards (`goal.a`) before every
   plan, so `C_obs` follows the real asymmetric silhouette instead of a circle
   around it.

   Planning with `goal.a` rather than the live heading matters. The drive
   controller turns towards `goal.a` while it follows the path, so planning from
   the live heading made the path a function of the very angle the controller
   was changing: a few degrees of rotation moved the inflation, the path jumped
   to the other side of an obstacle, and the robot oscillated left and right.
   `goal.a` is fixed for the whole move and is what the controller converges
   on, so the planner and the controller agree.

   That is the whole reason for using a polygon: an asymmetric robot can take a
   gap only its nose fits through. On a corridor test the triangle passes a
   360 mm slot when the goal heading points it nose-first, but needs 420 mm when
   the goal heading is sideways.

   The hull is grown by a small **5 mm clearance**, purely so the controller has
   room to correct against a hairline contact. It is *not* a heading margin:
   the heading is fixed for the whole move, so there is no drift to absorb.
   The clearance is applied by pushing each hull vertex out along its own
   direction, which keeps the triangle sharp and at 3 vertices. Do not swap it
   for `grow_polygon`: that arc-samples every corner, turning the hull into ~33
   vertices and roughly tripling the planner's edge tests, while rounding off
   exactly the corners that make tight gaps possible.

2. **Routing nodes.** The shortest path only bends at corners of the union of
   those polygons, so the nodes are the union boundary corners (plus the start
   and the goal).

3. **Search.** A* over the visibility graph between those nodes. An edge exists
   when the straight segment between two nodes stays outside every `C_obs`.

The table border is added as four walls inflated by the robot, so the path never
leaves the table. The four corners also carry protected zones, 300 mm along x
and 200 mm along y, which the robot may never touch.

## Code layout

| File | Contents |
|---|---|
| `navigation/geom.hpp/.cpp` | Vectors, convex polygons, distances |
| `navigation/convexPathfind.hpp/.cpp` | `C_obs`, nodes, A* |
| `navigation/pathfind.hpp` | The API used by the rest of the robot |
| `navigation/pathfind.cpp` | Turns the table state into obstacles |

The API is small:

```cpp
pathfindSetup(tableStatus);                         // start-up, and when stones move
pathfindUpdateOpponent(&lidar, tableStatus);        // every loop, cheap
int n = pathfind(start, goal, path, length_mm);     // 0 means no path
```

`pathfind()` returns 0 when the goal cannot be reached; callers already treat a
non-positive length as a failure.

## Web display

`GET /get_global` now returns a `pathfinding` object instead of the old
`costmap` grid:

```json
{
  "obstacles": [
    { "kind": "stone", "corners": [{"x": -160, "y": 145}, ...] },
    { "kind": "opponent", "corners": [{"x": 550, "y": -617}, ...] }
  ],
  "robot": [{"x": 230.9, "y": 0}, {"x": -115.5, "y": 200}, {"x": -115.5, "y": -200}],
  "inflated": [
    { "border": true,  "opponent": false, "corners": [...] },
    { "border": false, "opponent": true,  "corners": [...] }
  ],
  "path": [{"x": 0, "y": 1200}, ...],
  "found": true,
  "length_mm": 2790.0
}
```

- `obstacles` are the real objects on the table. Stones send their corners
  because they can be rotated.
- `robot` is the hull the planner actually used: rotated to the live heading and
  grown by the clearance. The frontend translates it to the current position and
  rotates it by the robot's heading, so the drawing matches what was planned.
- `inflated` is the region the robot's reference point may not enter, i.e.
  `C_obs = obstacle (+) (-robot)`. It is drawn as a translucent layer, which
  shows at a glance why the path keeps its distance. The four table walls come
  first with `"border": true`; the frontend strokes those instead of filling
  them, because they span the whole table. The opponent's shape carries
  `"opponent": true` so it can be tinted differently from the stones.

The same data is available alone at `GET /get_pathfinding`.

## Testing locally

A desktop build starts in test mode automatically (`TEST_API_ONLY` is defined
when not cross-compiling):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/programCDFR
```

In test mode every 500 ms the table is re-randomized (four stones at random
angles, a random opponent) and the robot and its target move and turn, so a new
path is planned each time. Open the display page to watch it.

Stones are kept in the central band and away from each other, and the start and
goal stay clear of the corner protection zones, so the generated tables are
solvable. A tighter layout is genuinely blocked for a 400 mm-wide robot and
would only show "no path".
