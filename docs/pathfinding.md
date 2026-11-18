# Pathfinding

The robot plans on the exact table, not on a grid. The robot is a convex
polygon, the stones are rotated rectangles and the opponent is a circle, so a
circle approximation is never used.

## How it works

1. **Configuration obstacles.** For each obstacle `O` the planner computes

   ```
   C_obs = O (+) (-robot)
   ```

   the set of robot positions that collide with `O`. It is a convex polygon and
   does not depend on the direction of travel.

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
    { "kind": "opponent", "x": 350, "y": -900, "r": 225 }
  ],
  "robot": [{"x": -250, "y": -125}, ...],
  "inflated": [
    { "border": true,  "corners": [...] },
    { "border": false, "corners": [...] }
  ],
  "path": [{"x": 0, "y": 1200}, ...],
  "found": true,
  "length_mm": 2790.0
}
```

- `obstacles` are the real objects on the table. Stones send their corners
  because they can be rotated.
- `robot` is the hull the planner uses, in the robot's own frame. The frontend
  translates it to the current position and rotates it, so the drawing matches
  what is actually planned for.
- `inflated` is the region the robot's reference point may not enter, i.e.
  `C_obs = obstacle (+) (-robot)`. It is drawn as a translucent layer, which
  shows at a glance why the path keeps its distance. The four table walls come
  first with `"border": true`; the frontend strokes those instead of filling
  them, because they span the whole table.

The same data is available alone at `GET /get_pathfinding`.

## Testing locally

A desktop build starts in test mode automatically (`TEST_API_ONLY` is defined
when not cross-compiling):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/programCDFR
```

In test mode every 500 ms the table is re-randomized (five stones at random
angles, a random opponent) and the robot and its target move, so a new path is
planned each time. Open the display page to watch it. The corridor in the
middle of the table is kept free of stones so a path is usually found.
