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

## Known limitation: the robot does not travel along its heading

The inflation above uses the hull at one heading, `goal.a`, and assumes that
heading describes the robot for the whole move. That assumption is weaker than
it looks, in three ways worth writing down before anyone builds on it.

**1. The robot strafes.** It can translate in any direction regardless of where
its nose points, and the planner never models the direction of travel. A
segment is only safe if the hull clears the obstacles at *every* translation
direction the controller might choose, and that is not checked.

**2. There is no single "worst" orientation.** A common assumption is that the
silhouette is widest at 45 deg. For this hull it is not. Measured width across
the axis of travel for the 400 mm triangle:

| heading | 0 deg | 30 deg | 45 deg | 60 deg | 90 deg |
|---|---|---|---|---|---|
| width (mm) | 346.4 | 400.0 | 386.4 | 346.4 | 400.0 |

The maxima are at 30, 90 and 150 deg, and 45 deg is a local *minimum*. Any
argument that reasons about "the worst heading" is shape-specific and does not
generalise.

**3. The shape is not fixed.** The robot grows when carrying stones, so the hull
is a function of load, not just heading. `buildRobotPolygon()` is still a
hard-coded triangle with a `TODO` about a single source of truth.

### Options, if this is not good enough

**A. Conservative bounding (the disc).** Inflate by the maximum reach over all
headings and all travel directions. A disc is isotropic, so a plan made this way
is safe for any shape, any strafe and any rotation.

It is *safe*, but it is not *usable* on its own. The radius is the circumradius
(235.9 mm with clearance), which is larger than the hull's reach in most
directions, so goals that only the real hull fits become unreachable. Measured
reach toward a wall against the disc radius:

| heading | 0 deg | 30 deg | 45 deg | 60 deg | 75 deg | 90 deg |
|---|---|---|---|---|---|---|
| hull reach (mm) | 204.3 | 118.0 | 166.8 | 204.3 | 227.9 | 235.9 |
| extra the disc demands (mm) | 31.6 | **118.0** | 69.1 | 31.6 | 8.0 | 0.0 |

The overhead is not a fixed amount: it is zero when the nose already points at
the obstacle and worst when the hull presents its narrow edge, up to 118 mm.
That is exactly the bug that made this the wrong default -- a goal flush with a
wall or a stone at the back of a quarry became unreachable. It was implemented
as a fallback and then removed.

So A is the correct answer to "never collide" and the wrong answer to "get
anywhere useful". Treat it as a safety net that a real solution falls back to,
never as the primary planner.

**B. Search over (x, y, theta).** Make orientation part of the search state. Each
segment is then inflated with the hull at its own heading, and the planner finds
where to turn instead of being told. Handles strafing because the search does
not assume travel along the heading. Costs roughly 8x the nodes; at 15 deg steps
that is about 1300 against a current peak of 157, so `kMaxNodes` would need
raising or the heading step coarsening.

**C. Inflate per travel direction.** Sweep the hull over the set of directions
the robot may translate in for a segment. Closest to the physical reality, but
the result is not a single convex polygon, so it needs per-direction checks or a
conservative convex hull of the union. More machinery than B for a similar
outcome.

**D. Model the real hull per load state.** Whatever the algorithm, it is only as
good as the shape it is given. Nothing else gets more accurate until this
exists.

**Rejected: a linear heading profile along the path.** Assigning theta from
`start.a` to `goal.a` by arc length, then re-validating, looks cheap and is
tempting. It does not work well here: it places mid-rotation wherever the path
happens to be halfway, it cannot correct *where* the robot turns (only report
that the result failed), and it assumes the shape is a known function of a
single angle, which points 1 and 3 above contradict.

Recommended order if this becomes a priority: **D** first (the hull is the input
to everything), then **B**. A is sound but costs too much reach to use as the
main planner -- see above.

## Code layout

| File | Contents |
|---|---|
| `navigation/geom.hpp/.cpp` | Vectors, convex polygons, distances |
| `navigation/convexPathfind.hpp/.cpp` | `C_obs`, nodes, A* |
| `navigation/pathfind.hpp` | The API used by the rest of the robot |
| `navigation/pathfind.cpp` | Turns the table state into obstacles |

The API is small:

```cpp
pathfindSetup(tableStatus);                         // start-up: remembers the table
pathfindUpdateOpponent(&lidar, tableStatus);        // every loop, cheap
int n = pathfind(start, goal, path, length_mm);     // 0 means no path
```

`pathfind()` rebuilds the stones and wall areas from the table on **every call**,
so nothing has to notice when a quarry is emptied or a stone is dropped: picking
a stone up simply stops it being an obstacle on the next plan. Building the
obstacle set is a few small polygon constructions and measures far below the
cost of the A* that follows, so there is no reason to cache it.

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
