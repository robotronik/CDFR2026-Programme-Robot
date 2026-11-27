# Camera

Everything the robot does with its downward camera: where it is mounted, how it
is calibrated, how a frame becomes a field position, and where the tuning
constants live.

## Constraints (read first)

- **Accuracy target: 3 mm.** That is good enough for this robot; do not chase
  sub-millimetre precision at the cost of speed or robustness.
- **Feature grid: 1.5 mm/px.** Chosen as the speed/accuracy knee. Coarser
  (2–3 mm/px) is faster but loses the harshest low-light captures; finer
  (1–1.25 mm/px) costs much more without solving more.
- **Keep it fast.** On x86_64 a frame costs ~38 ms through the feature
  localiser (windowed by the odometry prior). The camera thread also runs ArUco
  (~8 ms); do not run the feature localiser on every frame unless the budget
  allows it (see *Performance*).

## Hardware

- **OV9281** monochrome global-shutter sensor, 1280×800, mounted looking down at
  the table and pitched forward.
- On the Pi 5 the sensor sits behind the PiSP ISP, so it is driven through
  **libcamera** (`Cam` → `vision::ArucoDetector` → `LibcameraCamera`). The
  capture thread is the only thread that touches the device.
- Default image controls: exposure EV 1.5, contrast 1.3, brightness 0 (see
  `ArucoDetector`), adjustable at runtime through `Cam`.

## Calibration files

- `data/OV9281_1280_800.yaml` — OpenCV intrinsics (`camera_matrix`,
  `dist_coeffs`) for the OV9281 at 1280×800. Used to undistort and to build the
  ground homography.
- `tests/data/features/FieldBW.png` and `data/FieldBW.png` — the top-down field
  map. **1.00000 px per mm**, 2000×3000 px = 2000×3000 mm, origin at the image
  centre. It is the field design at true scale, matching the printed vinyl: the
  four landmark tags land within 0.6 mm of their design positions, so the map is
  the metric reference (no separate scale factor).

## Mounting geometry

Defined in `vision/CameraGeometry.hpp`, measured from the landmark-tag poses:

| constant | value | meaning |
|---|---|---|
| `CAMERA_HEIGHT_MM` | 217 | lens height above the table |
| `CAMERA_PITCH_DEG` | 43.1 | downward tilt of the optical axis |

- **Height is benign.** It only scales the ground homography uniformly, and the
  matcher fits scale freely (gated 0.9–1.1), so a height error is absorbed and
  does not bias the position.
- **Pitch is the sensitive one.** A pitch error is a keystone (scale varies with
  image row) that a similarity cannot absorb; ~2–4 mm of position shift per
  degree. It was re-measured to 43.1° (median) when the camera was repositioned.

## Camera ↔ robot frame

The camera is offset from the robot's rotation centre. `cameraToRobot()` /
`robotToCamera()` (in `Cam.hpp`) convert between them using:

| constant | value |
|---|---|
| `OFFSET_CAM_X` | 129 mm |
| `OFFSET_CAM_Y` | 4.5 mm |
| `OFFSET_CAM_A` | 0° |

These are recovered with `calibrate_camera()` (see below) and should be compared
against a caliper measurement of the mount after any mechanical change.

## Localisers

Both run on the same capture thread in `Cam::workerLoop`. `Cam::getLocalisation`
returns the latest camera pose on the table (use `cameraToRobot` for the robot
pose).

### ArUco (landmark tags)

- Four fixed tags, **ids 20–23**, 100 mm squares, at the table corners
  (±400, ±900) mm — `vision::ArucoLocalizer::landmarkTags()`.
- One visible tag gives a fix; several are averaged (heading circularly).
- A single planar tag has a weak depth axis, so one tag is only good to roughly
  5–15 mm. Treat ArUco as the robust, coarse reference, not the precise one.

### Features (map matching)

`vision::FeaturesLocalizer`, tuned for speed at 1.5 mm/px. Pipeline per frame:

1. **Ground warp + undistortion, folded into one remap.** A capture→patch remap
   (undistort and ground homography composed) is precomputed once per frame size
   and applied with a single `cv::remap`, so a frame is resampled once, not
   twice. The patch is a top-down view at 1.5 mm/px.
2. **Patch crop (`kPatchCrop = 0.85`).** The camera footprint is a trapezoid, so
   the rectangle around it is ~45 % black; a centred 0.85 crop keeps the usable
   ground and cuts AKAZE cost, which scales with patch area.
3. **Illumination normalisation.** A box high-pass (`kHighpassKernel = 15`)
   removes glare gradients, then CLAHE (`kClaheClip = 1.5`, 8×8 tiles) stretches
   the local contrast. The **map is normalised the same way** so descriptors are
   comparable. This is what makes the low-contrast markings stable across
   lighting.
4. **AKAZE** (MLDB, 486-bit descriptors, threshold `kAkazeThreshold = 2e-4`).
   The threshold is high because normalisation lifts the markings well above it,
   which keeps the keypoint count — and the matching cost — low.
5. **Match** against map keypoints restricted to the odometry prior's view
   window, with `BFMatcher` + Lowe ratio (`kRatioTest = 0.95`) + mutual
   cross-check.
6. **Rigid fit + safety gates.** `estimateAffinePartial2D` (RANSAC). A fix is
   only accepted with ≥ `kMinInliers = 15` consistent matches, scale 0.9–1.1 and
   an inlier span ≥ `kMinInlierSpanMm = 300` mm. These gates prefer *no fix* over
   a *wrong* fix (repetitive texture and the field's near-symmetry can otherwise
   produce a confident 180° flip).

Filtering is done **after** the warp, not before: a fixed kernel only has a
uniform physical scale in the rectified patch, and the map can only be
normalised there. Filtering the raw frame beforehand drops accuracy sharply.

## Accuracy

- **Target: 3 mm.** Met, with margin, in the current test set.
- Inlier residual RMS ~2.5 mm (median) across the test captures — this is the
  internal consistency of a fix, dominated by map/geometry rather than noise.
- Agreement with the ArUco localiser ~7 mm median (worst ~19 mm), but ArUco
  (single tag) is the less precise side, so this is an upper bound on the
  feature error. Expected per-frame accuracy ~2–3 mm, ~0.3–0.5° heading; ~5 mm is
  a safe working bound.
- Absolute accuracy is ultimately capped by how faithfully `FieldBW.png` matches
  the printed vinyl and by the mounting constants; both are anchored (map is
  true scale, tags verified to <0.6 mm).

## Performance

Per frame on x86_64 (Release), same machine that builds the tests:

| stage | cost |
|---|---|
| folded remap | ~2–3 ms |
| illumination normalisation | ~1 ms |
| AKAZE detect | ~26 ms |
| match (prior window) | ~5 ms |
| RANSAC | ~1 ms |
| **total (windowed)** | **~38 ms** |

Detection dominates and scales with patch area. On the Pi 5 expect roughly 3–4×
this. Because ArUco is cheaper and more robust when a tag is visible, the
recommended in-match use is the feature localiser as a **fallback / lower-rate**
fix rather than every frame.

## Calibrating the camera mount — `calibrate_camera()`

`actions::calibrate_camera()` (in `calibration.cpp`) is a manual routine to run
after moving the camera. It is a stepped function for the main FSM:

- The robot **rotates in place** so the camera looks at the two landmark tags on
  its colour side, sampling several headings; it then returns to the start
  heading.
- Because the robot does not translate, comparing camera poses across the sweep
  with the raw OTOS heading recovers the **camera offset in the robot frame**
  (x, y, a) — independent of odometry position error.
- It also logs the **camera height** and the mount's **Euler rotation**
  (roll/pitch/yaw) from the tag poses.

Results are logged (`LOG_GREEN_INFO`); update `OFFSET_CAM_*` (and the mounting
constants if the height/pitch changed) from them.

To validate accuracy with no ground truth, use the same idea: rotate in place,
fit the circle the camera traces, and compare its radius to a caliper-measured
camera offset. Residuals give the precision; the radius check gives absolute
scale.

## Tests and data

- `tests/data/aruco_loc/` — real captures for the ArUco localiser.
- `tests/data/features/` — real captures for the feature localiser, plus
  `FieldBW.png`.
- `tests/data/aruco_blocs/` — synthetic captures for the game-element pose.

Run:

```bash
./build.sh test                                   # whole suite
docker run ... ./build/x86_64/robot_tests aruco    # ArUco only
docker run ... ./build/x86_64/robot_tests features # features only
```

Current results: **features 42/44**, **ArUco 11/11** — both pass, with no wrong
poses against the ArUco cross-check.

## Tuning knobs (`vision/FeaturesLocalizer.cpp`)

| constant | value | effect |
|---|---|---|
| `kMmPerPx` | 1.5 | working grid; the main speed/accuracy lever |
| `kPatchCrop` | 0.85 | fraction of the footprint kept; smaller = faster |
| `kHighpassKernel` | 15 | glare-removal box size |
| `kClaheClip` | 1.5 | local contrast strength |
| `kAkazeThreshold` | 2e-4 | keypoint response floor |
| `kRatioTest` | 0.95 | Lowe ratio for matches |
| `kRansacMm` | 8 | inlier tolerance |
| `kMinInliers` | 15 | minimum consensus (>10; prefer no fix) |
| `kMinInlierSpanMm` | 300 | minimum inlier spread (rejects tight clusters) |
| `kPriorYawSlackDeg` | 15 | prior window heading slack |

## Alternatives considered

Keep AKAZE + normalisation; these were measured and rejected:

- **ORB**: 0–2/44 — its descriptors do not match the synthetic map.
- **Coarse-detect / fine-describe**: detection at 2–3 mm collapses the solve
  rate (fine resolution is what finds the markings).
- **Direct alignment (`findTransformECC`)**: needs the extra `opencv-video`
  module, is much slower (~150 ms) and did not converge on this
  photo-vs-synthetic-map pair.
