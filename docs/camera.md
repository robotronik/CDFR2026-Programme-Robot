# Camera

Downward camera: mounting, calibration, localisation, tuning.

## Constraints

- **Latency: under 150 ms per frame on the Raspberry Pi 5.**
- **Lighting robustness**: must work under glare and in low light.
- **Accuracy: 3 mm is sufficient** — no need for anything more precise.

The implementation choices are consequences of these, not requirements: the
1.5 mm/px grid and 0.85 crop are what fit the latency budget at 3 mm, and the
high-pass + CLAHE normalisation is what delivers lighting robustness.

## Hardware and files

- OV9281 mono, 1280×800, driven through **libcamera** on the Pi 5.
- `data/OV9281_1280_800.yaml` — OpenCV intrinsics.
- `data/FieldBW.png` — top-down map, **1.00000 px/mm**, 2000×3000 mm, origin at
  image centre, greyscale. Rendered from the print file (`table.svg`, Inkscape,
  3000×2000 mm) at 1 px/mm, rotated 90° CCW, desaturated — so it is the printed
  vinyl, metric by construction: the landmark tags land at exactly
  (±400, ±900) mm.

## Mounting

In `vision/CameraGeometry.hpp`, measured from tag poses:

| constant | value |
|---|---|
| `CAMERA_HEIGHT_MM` | 217 | height only scales uniformly (absorbed by the fit) |
| `CAMERA_PITCH_DEG` | 43.1 | sensitive: ~2–4 mm shift per degree |

Camera↔robot offsets (`Cam.hpp`), from `calibrate_camera()`:
`OFFSET_CAM_X = 129`, `OFFSET_CAM_Y = 4.5`, `OFFSET_CAM_A = 0` (mm/deg).

## Localisers

`Cam::workerLoop` runs both; use `cameraToRobot()` for the robot pose.

- **ArUco** (`ArucoLocalizer`): tags **20–23**, 100 mm, at (±400, ±900) mm. One
  tag is enough (averaged if several); a single tag is only good to ~5–15 mm.
- **Features** (`FeaturesLocalizer`), per frame:
  1. one folded capture→patch `remap` (undistort + ground warp, precomputed);
  2. crop to `kPatchCrop` of the footprint (it is ~45 % black);
  3. illumination normalise — box high-pass + CLAHE, on map *and* patch;
  4. AKAZE (MLDB 486, 2 scale layers) at `kAkazeThreshold`;
  5. match against the odometry-prior window (BF + ratio + mutual);
  6. RANSAC partial affine, gated by min inliers, scale, inlier span and
     heading vs the prior — prefer no fix over a wrong one.

Filter **after** the warp: a fixed kernel only has uniform scale there.

## Accuracy and performance (x86_64, Release)

- Residual ~2.5 mm median; expect ~2–3 mm, 5 mm bound. ArUco agreement ~7 mm
  (ArUco is the less precise side). Map is the fixed reference.
- ~20 ms/frame windowed on x86_64, AKAZE detection dominating. The Pi 5 runs
  ≈3–4× slower, i.e. within the 150 ms budget (confirm on device).
- The prior heading is what makes AKAZE's cheap 2-layer pyramid safe: a
  180° mise-match (the field texture is near-symmetric) is a valid rigid motion,
  so without a prior it can pass every other gate. None in prior mode; one in
  whole-map mode.

## `calibrate_camera()`

Manual (`calibration.cpp`): the robot rotates in place to view the two
colour-side tags, then logs the camera offset (x, y, a), height and Euler
rotation. Run after moving the camera; update `OFFSET_CAM_*` from the output.

## Tests

`tests/data/{aruco_loc,features,aruco_blocs}`; `./build.sh test` (or
`robot_tests aruco|features`). Both are strict: **every** capture in the set must
localise (**features 42/42**, **ArUco 11/11**), so a single miss is a failure.
`tests/data/features_no_field` holds two bare-table frames that are deliberately
not in the set — they show only a sliver of field, and a fix cannot be supported.

The features test runs whole-map (no prior), so one of the 42 fixes is a 180°
mise-match (see above); the test checks coverage, not pose sanity.

## Tuning (`FeaturesLocalizer.cpp`)

| constant | value |
|---|---|
| `kMmPerPx` | 1.5 |
| `kPatchCrop` | 0.85 |
| `kHighpassKernel` / `kClaheClip` | 15 / 0.5 |
| `kAkazeThreshold` / `kRatioTest` | 2e-4 / 0.95 |
| `kAkazeOctaveLayers` | 2 |
| `kRansacMm` / `kMinInliers` / `kMinInlierSpanMm` | 8 / 15 / 300 |
| `kPriorYawSlackDeg` / `kPriorHeadingToleranceDeg` | 15 / 25 |

AKAZE's default four scale layers add no keypoints on this map but cost 13 ms;
two layers give the same solves and the same poses (median 1 mm, max 6.5 mm) at
~25 ms. Raising `kAkazeThreshold` to cut the cost further loses the harder
captures (4e-4 → 41/44 whole-map, 8e-4 → 35/44).

`kClaheClip` 0.5 beats the old 1.5: it stops CLAHE amplifying noise in the flat
panels, so the map keeps fewer, cleaner keypoints — every capture solves in prior
mode at ~20 ms against 43/44 at ~25 ms, with the same accuracy against ArUco
(9–10 mm mean either way). The price is geometric support without a prior
(42/44 and one flip, against 43/44 clean). The normalisation itself is
load-bearing: raw frames solve 34/44, high-pass alone 40/44, CLAHE alone 39/44.

Alternatives tried, each at its best setting (whole-map solves/44; ms is the
per-frame whole-map cost). None beat AKAZE (~20 ms windowed, everything solved in
prior mode):

| detector | solves | ms |
|---|---|---|
| BRISK | 40/44 (or 32/44 coarse, 19 ms) | 166 |
| KAZE | 40/42 windowed | 148 |
| SIFT | 38/42 windowed | 47 |
| ORB (density matched) | 37/44 | 220 |

Also rejected: coarse-detect/fine-describe; ECC direct alignment (slower, did
not converge). Watch out with any `nfeatures`-style cap: it applies to the whole
map, so an under-set cap benchmarks the cap, not the detector.
