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
- `data/FieldBW.png` (= `tests/data/features/FieldBW.png`) — top-down map,
  **1.00000 px/mm**, 2000×3000 mm, origin at image centre. True scale matches the
  printed vinyl (tags land within 0.6 mm), so it is the metric reference.

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
  4. AKAZE (MLDB 486) at `kAkazeThreshold`;
  5. match against the odometry-prior window (BF + ratio + mutual);
  6. RANSAC partial affine, gated by min inliers, scale and inlier span —
     prefer no fix over a wrong one.

Filter **after** the warp: a fixed kernel only has uniform scale there.

## Accuracy and performance (x86_64, Release)

- Residual ~2.5 mm median; expect ~2–3 mm, 5 mm bound. ArUco agreement ~7 mm
  (ArUco is the less precise side). Map is the fixed reference.
- ~38 ms/frame windowed on x86_64: remap ~3, normalise ~1, AKAZE detect ~26,
  match ~5, RANSAC ~1. The Pi 5 runs ≈3–4× slower, i.e. within the 150 ms
  budget (confirm on device).

## `calibrate_camera()`

Manual (`calibration.cpp`): the robot rotates in place to view the two
colour-side tags, then logs the camera offset (x, y, a), height and Euler
rotation. Run after moving the camera; update `OFFSET_CAM_*` from the output.

## Tests

`tests/data/{aruco_loc,features,aruco_blocs}`; `./build.sh test` (or
`robot_tests aruco|features`). Current: **features 42/44**, **ArUco 11/11**.

## Tuning (`FeaturesLocalizer.cpp`)

| constant | value |
|---|---|
| `kMmPerPx` | 1.5 |
| `kPatchCrop` | 0.85 |
| `kHighpassKernel` / `kClaheClip` | 15 / 1.5 |
| `kAkazeThreshold` / `kRatioTest` | 2e-4 / 0.95 |
| `kRansacMm` / `kMinInliers` / `kMinInlierSpanMm` | 8 / 15 / 300 |

Alternatives: ORB (with comparable map density ~35/44 and ~6× faster
detection, but it loses the low-light fixes), coarse-detect/fine-describe, and
ECC direct alignment (slower, did not converge).
