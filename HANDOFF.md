# Handoff: CSE 276A HW4 (path planning: max-safety vs. min-time)

Last updated 24 Nov 2026 (Pacific). HW3 is below, kept for reference; HW4 is the active work.

**HW4 deadline: 25 Nov 2026, before midnight** (`CSE276A_26_HW4.pdf`). **Current focus:** physical
setup (central obstacle + 4 new tags), the two robot runs, then report + zip. HW3's deadline (13
Nov) already passed; HW3's own report fixes (video link, code zip) are still open separately, see
its section below.

## HW4: path planning in the 8x8 ft arena (reusing HW3's arena and HW2's known-map localization)
HW4 is **localization with a known map**, not SLAM — it reuses `closed_loop_follower.py` /
`localization.py` from HW2 unchanged, fed two different planned paths. No new ROS logic was
needed, only a new offline planner. See the approved plan for full reasoning:
`/Users/sohamlokhande/.claude/plans/continue-planning-the-above-ticklish-goblet-of-9dd93bcb-9e6d-482b-bfe4-6d8ac38c928f.md`.

**Status**
| Item | State |
|---|---|
| Arena | Reusing HW3's 8x8 ft (2.438 m) square, but the 8 edge tags (ids 0-7) are **repositioned** per the HW4 figure: clustered in pairs near each of the 4 corners (12 in in from the corner along each wall), not at the HW3 quarter-points. |
| Central "obstacle" | **No physical box.** The user chose to mark the obstacle with 4 tags (8-11), one at the midpoint of each side of an 18 in (0.4572 m) virtual-obstacle square (within the assignment's 1x1-2x2 ft range), centered at arena-center (4 ft, 4 ft) = (1.2192, 1.2192) m, spanning 39-57 in on each axis, each facing straight outward along a cardinal direction (not diagonal). **Not yet physically placed.** The planner treats this square as blocked regardless of there being a real box there. |
| New tags 8-11 | **Print `apriltags_8_11.pdf`** (repo root, user's preferred source for these 4 pages over `printables/apriltags.pdf` pages 9-12 — same tag36h11 ids 8-11, both decode-verified correct). Mount each on a small stand/post at its side-midpoint, facing straight outward (E/N/W/S). **Not yet printed/placed.** The PNGs backing `printables/apriltags.pdf` were generated with `cv2.aruco.generateImageMarker(..., borderBits=1)` + a 180° rotation to match the existing tags' bit layout (`tools/make_printables.py`, `TAG_FILES` now covers ids 0-11). |
| `apriltag_ros/cfg/tags_36h11.yaml` | Updated to include ids 8-11, size 0.1435 (same as tags 0-7) as a placeholder. **Re-measure the actual printed black-square size of tags 8-11 after printing `apriltags_8_11.pdf`** (it may scale slightly differently from the original 0-7 print run) and update their `sizes` entries if it differs from 0.1435. **Needs `colcon build --packages-select apriltag_ros` on the robot**, not just `robot_control`, since this is a config file in that package. |
| `landmarks_hw4.txt` | Rewritten: tags 0-7 at their new corner-clustered positions, tags 8-11 at the virtual-obstacle square's 4 side-midpoints, cardinal facing. **All 12 positions are planned, not measured** — remeasure every one once placed, the same way HW3's tags were. |
| `tools/planners.py` | **New, working.** Two planners sharing one obstacle/robot-radius model: max-safety (occupancy grid + BFS distance transform + Dijkstra weighted by `1/clearance`) and min-time/distance (visibility graph over the inflated obstacle's corners + Dijkstra). Verified end to end on the Mac (no ROS): `tools/.venv/bin/python tools/planners.py --start 0.4064 0.4064 45 --goal 2.032 2.032 45` writes `waypoints_hw4_safety.txt`, `waypoints_hw4_time.txt` and `report_hw4/figures/{safety,time}_plan.png`. |
| Waypoints files | Generated at the repo root with start (16 in, 16 in, 45°) = (0.4064, 0.4064, 45°) and goal (80 in, 80 in, 45°) = (2.032, 2.032, 45°) — adjust and rerun `planners.py` if the real obstacle/start/goal differ from the plan once placed. **Important:** these files must stay pure `x, y, theta` lines with no `#` comments — `waypoint_follower.load_waypoints` (used by `closed_loop_follower`) does not strip comments, unlike `ekf_slam.load_path`. |
| `hw4_tmux.sh` | **New**, modeled on `hw2_tmux.sh`: `./hw4_tmux.sh safety` or `./hw4_tmux.sh time`, same 4-pane camera/apriltag/motor/follower layout, logs to `~/hw4_runs/logs/`. |
| Robot runs | **Not done yet.** Need one run per mode, each filmed, each measured against the planned goal with a tape. |
| Report (`report_hw4/`) | Figures directory created by `planners.py`; the report document itself is not started. |
| Video / code zip | Not done. |

**Next steps for the new session**
1. Place the obstacle, tape tags 8-11 to its faces, remeasure all 12 positions into `landmarks_hw4.txt`, rerun `tools/planners.py` with the real obstacle rectangle/start/goal if they differ from the plan's.
2. Pull + `colcon build --packages-select apriltag_ros robot_control` on the robot, confirm tags 8-11 are detected.
3. Run `./hw4_tmux.sh safety` then `./hw4_tmux.sh time` from the start X, filming each, measuring the true stop position.
4. Plot each run's CSV with `tools/plot_trajectory.py --waypoints waypoints_hw4_<mode>.txt --landmarks landmarks_hw4.txt`.
5. Write the report (architecture, both representations/algorithms with motivation, the 4 figures, measured error, video links) and build the code zip (same recipe as HW2/HW3).

## HW3 (reference; its own deadline already passed 13 Nov 2026)

**Deadline was Friday 13 Nov 2026, before midnight.** The assignment is `hw3-2026.pdf` and the lecture decks are `lec8a-2026.pdf` (Kalman filter / EKF) and `lec8b-2026.pdf` (SLAM). Kumaran's report draft is `CSE_276A_HW3.pdf`. All of these are in the repo root and untracked.

**HW3's own remaining focus (lower priority than HW4 right now): finish its report and submission (report fixes, video link, code zip). v3 comes later.**

HW2 is finished; see "HW2 (done)" at the end.

### Status at a glance
| Item | State |
|---|---|
| Arena and ground truth | Done. The tags are at the planned positions, which the user confirmed are the measured ground truth (`ground_truth_hw3.txt`). |
| SLAM code | **v2 on `main` is final for the report:** 25° turn pieces, TURN_GAIN 0.9, TURN_STD_FRACTION 0.35. v3 is on the **local** branch `hw3-v3` (commit `e68b974`), not pushed and never run on the robot. See "v3 (later)". |
| Square run | **Done: square run 3**, in `report_hw3/square_run3/`. The figures are a v2 replay. Error after loops 1/2/3: 16.2 / 6.9 / **9.4 cm** against the mark, 7.1 / 5.3 / **5.5 cm** best-fit; 0 rejections. **The robot drove it with older code** (46° turn pieces, `TURN_SCALE=0.8`, turn std 0.08). Only the filter in the figures is v2. |
| Octagon run | **Done: octagon run 4** (`bundle_20261113_221216`, v2 on the robot, `TURN_SCALE=1.0 DRIVE_SCALE=0.92 DRIFT=6`), in `report_hw3/octagon_run4/`. Error after loops 1/2/3: 19.2 / 13.8 / **14.0 cm** against the X, 10.8 / 4.5 / **3.8 cm** best-fit; 0 rejections; 7 of 8 tags. The uniform ~12 cm shift looks like start placement: the fitted start is (0.874, 0.602) against the X at (0.991, 0.667). **The user still hasn't measured where the X really is.** |
| Report data pushed | Commit `3d796dc` on `origin/main`: both run folders plus `report_hw3/diagnosis/failed_octagons_actual_vs_planned.png`. |
| Report | **Kumaran wrote `CSE_276A_HW3.pdf`.** It was reviewed on 24 Nov and needs the fixes listed under "Report review" below. |
| Video | **Missing.** The report has `https://youtu.be/TODO`. The user films the runs and uploads an unlisted video. |
| Code zip | **Not built yet.** Same recipe as HW2 (see "HW2 (done)"), built from `main`. |
| Lens-to-axle distance | Never measured. `CAMERA_OFFSET = 0.10` m is a guess; the replay can correct it with `--camera-offset`. |

## Report review (24 Nov): what's left to fix in `CSE_276A_HW3.pdf`
The algorithm equations (state, z, F, G, H, Σ₀), the noise values, the gate thresholds, the recovery rule, and Tables 3–5 all match the code and the summary JSONs. The report covers every section the assignment asks for. Still to do:

**Must fix**
1. **Video link:** replace `https://youtu.be/TODO` with the real link.
2. **Say how the square run was produced.** Section 2 describes 25° pieces and κ = 0.9, but the square run was *driven* with 46° pieces and turn scale 0.8 (see the log header in `report_hw3/square_run3/slam_20261113_062025.jsonl`). The square map is the final filter re-run offline on the logged motions and sightings. Suggested sentence: "The square run was recorded with an earlier version of the planner (turn pieces up to 46°, turn scale 0.8); the map and trajectory shown are the final filter (Section 2) re-run offline on that run's logged motions and tag sightings. The octagon run used the final code on the robot."
3. **Explain why the square's error rises in loop 3** (6.9 → 9.4 cm against the mark). The whole map rotated from −1.9° to −3.2° relative to the start frame while its shape stayed the same (best-fit 5.3 → 5.5 cm).
4. **Correct the sighting counts.** Square: tags 3/5/7 were seen **7/11/10** times and tags 0/2/4/6 **18–23** times (the report says 7–9 and 18–22). Octagon: 13–17 is correct (0:17, 2:15, 3:17, 4:14, 5:13, 6:17, 7:14).

**Should add**

5. **Estimated map vs ground truth** (assignment item 4). The square map is rotated about −3°, and its worst tags are 0 and 5 (15–17 cm). The octagon map has nearly the right shape (3.8 cm best-fit, 0.3° rotation) but is shifted by about 12 cm. Quote the fitted start pose (0.874, 0.602) against the mark at (0.991, 0.667).
6. **Calibration values:** drive scale 0.92, drift 6°/m, turn scale 1.0 (octagon) or 0.8 (square), and the HW1 command model in `calibration.py`.
7. **Name the detector:** `apriltag_ros` on rectified images from `robot_vision_camera`.
8. **Limitations paragraph.** The same turn command gave 0.56× to 1.48× the planned turn across sessions. That broke octagon runs 1–3, and it is why turns are split into 25° pieces with 35% turn noise. Use `report_hw3/diagnosis/failed_octagons_actual_vs_planned.png`.

**Small fixes**
- **Eq. 6:** write the heading noise as (4°/m)·|d|.
- **Eq. 5:** say that only the first 2 rows of H and R are used when ϕ is dropped beyond 2.2 m.
- **Figures 3a and 5a:** too small to read. Enlarge them or drop them.
- **Path clearance:** replace "0.60 to 2.0 m from the walls" with the real minimum clearances: 0.76 m for the square, about 0.67 m for the octagon (bottom wall).

**Next steps for the new session**
1. Help the user and Kumaran apply the fixes above. If the user gives the measured octagon X, replay with `--start X Y 0` into `report_hw3/octagon_run4/` and update Tables 3 and 5.
2. Build the code zip from `main` using the HW2 recipe (implementation folders only; comments stripped in the zip copy only; add the final PDF; exclude `__pycache__`). Name it like HW2's: `hw3_Soham_Lokhande_and_Kumaran_Karthik.zip`, in `/Users/sohamlokhande/dev/CSE276/`.
3. Only after submission: v3 (see "v3 (later)").

## What HW3 asks for (summary of `hw3-2026.pdf`)
1. An 8 ft × 8 ft arena (the minimum size) with 2 landmarks per side, and a measured ground-truth map with a sketch.
2. An EKF-SLAM system, as in the lectures. Landmarks are detected with off-the-shelf software (`apriltag_ros`).
3. The robot starts at an unknown pose with **no knowledge of the landmark positions or how many there are**. It drives (i) a square of n × n ft, then (ii) an octagon with side m ft.
4. Compare maps: transform the SLAM map into the world frame using the measured start pose, report the average landmark error, and say whether more loops help.
5. Report sections:
   - the algorithm: x and z; F, G, H; x₀ and Σ₀; Q and R; how new, out-of-view and reappearing landmarks are handled;
   - results for the square and the octagon;
   - how n and m were chosen, and the trade-offs;
   - the video link.
6. Plots of the estimated trajectory and the map, with **covariance ellipses** per landmark.
7. A zip of the code.

## Arena (8 ft = 2.438 m)
**Frame:** the origin is the bottom-left corner, +x runs along the bottom edge, +y up the left edge. Each tag sits on the edge line, at camera-lens height, facing inward. All tags are 36h11 and 14.35 cm, printed at the same 94% scale as before.

| Tag | Wall | Position (m) | Faces |
|---|---|---|---|
| 0, 1 | bottom | (0.813, 0), (1.626, 0) | +y |
| 2, 3 | right | (2.438, 0.813), (2.438, 1.626) | −x |
| 4, 5 | top | (1.626, 2.438), (0.813, 2.438) | −y |
| 6, 7 | left | (0, 1.626), (0, 0.813) | +x |

**Start marks.** The robot's **turning centre** (the midpoint between the drive wheels) goes on a taped X. The robot faces **+x**, along a taped line parallel to the bottom edge, pointing at the right wall (tags 2/3).
- **Square:** (0.762, 0.762), i.e. 30 in from the left edge and 30 in from the bottom.
- **Octagon:** (0.991, 0.667), i.e. 39 in from the left edge and 26¼ in from the bottom.

**What the start tag reading should be.** From the start pose, tag 2 should appear at forward f and left l (the camera offset is already included in f):
- **Square:** f = 1.68, l = +0.05. Measured in runs: f = 1.71–1.72, l = +0.08–0.09. Close; the ~4 cm in f is probably the lens offset.
- **Octagon:** f = 1.45, l = +0.15.
  - Octagon run 1 read 1.71 / 0.09: the robot was on the **square** mark by mistake (the user confirmed).
  - Octagon run 2 read **1.62 / 0.19**: about 13–17 cm too far back. Possibly the robot's front or camera, not its axle centre, was on the X. **Ask the user to re-check this.** Use the measured start pose when replaying.

**Paths.** The files are relative to the start pose. Both paths are 3.66 m long, run counter-clockwise, and are centred in the arena.
- `path_hw3_square.txt`: n = 3 ft (0.914 m).
- `path_hw3_octagon.txt`: m = 1.5 ft (0.457 m).

The robot never reads the tag map. `ground_truth_hw3.txt` is used only by the Mac tools.

## Code (v2, on `main`)
| File | What |
|---|---|
| `robot_control/robot_control/ekf_slam.py` | Pure-numpy EKF-SLAM, plus helpers shared by the robot node, the simulator and the replay (`fuse_look`, `next_motion`, `load_path`, `ellipse`). |
| `robot_control/robot_control/slam_follower.py` | ROS node `slam_follower`: stop-and-go along the path for N loops, an EKF update at every stop, writes `~/hw3_runs/slam_<time>.jsonl`. |
| `hw3_tmux.sh` | Robot launcher with 4 panes (camera, apriltag, follower, motor). Usage: `[TURN_SCALE=.. DRIVE_SCALE=.. DRIFT=..] hw3_tmux.sh square\|octagon [loops]`, or `hw3_tmux.sh noise [s]`. The follower is the **bottom-left pane**; press Enter there. |
| `hw3_bundle.sh` | Packs the newest run file and the newest log folder that has a `follower.log` into `~/hw3_runs/bundle_<time>.tgz`, and prints the scp command. |
| `apriltag_ros/cfg/tags_36h11.yaml` | IDs 0–7, size 0.1435. This is detector config only, not a map. |
| `tools/slam_replay.py` | Re-runs the EKF on a log and writes `_map.png` (start frame and arena frame, 95% ellipses), `_landmarks.png` (per-tag zoom for each loop), `_loops.png` and `_summary.json`. It prints the error against ground truth per loop, both with the measured start pose and with a best-fit (Kabsch) alignment. Options: `--start X Y THETA_DEG`, `--camera-offset`, `--kt` (extra factor on the turn gain), `--kd`, `--drift` (deg/m), `--noise` for noise logs. |
| `tools/slam_fit_motion.py` | Grid-fits turn scale, drive scale and drift by minimising the filter's mean NIS (normalized innovation squared). It uses no ground truth. |
| `tools/slam_measure_turns.py` | Measures actual turn angles directly, with no filter, from the change in tag bearing and yaw across a turn. |
| `tools/slam_sim.py` | Simulates a run and writes a log in the robot's format. Options: `--turn-bias`, `--turn-spread` (erratic turns: each turn is multiplied by a uniform factor in [1 − s, 1 + s]), `--seed`. |

**EKF design.** This is the report's "algorithm" section; give it to Kumaran.
- **State:** s = [x, y, θ, L1x, L1y, A1, …]. A_j is the direction tag j's face points. The map frame is the robot's start pose. x₀ = 0 and Σ₀ = 0 (3 × 3).
- **Motion, drive d:** x += d·cos θ', y += d·sin θ', θ += drift·|d|, where θ' = θ + drift·|d|/2. The Jacobian F is the identity on the landmarks, and Q is mapped into the state through W.
- **Motion, turn a:** θ += TURN_GAIN·a.
- **Measurement:** z = (f, l, φ), where f = cos θ·dx + sin θ·dy, l = −sin θ·dx + cos θ·dy and φ = A_j − θ. φ is not used beyond 2.2 m. H is in `predict_measurement`.
- **New landmark:** augment the state with L = p + Rot(θ)·z and A = θ + φ. The covariance comes from the Jacobians Gr and Gz, including the cross terms ("Strategy 2" in lecture 8b).
- **Out of view:** nothing changes; the landmark keeps its estimate and correlations.
- **Reappearing:** a normal update, which is the loop closure.
- **Gate:** Mahalanobis, χ² at 99.9%.
- **Lost-robot recovery:** after 2 looks in a row where every known tag is rejected, the robot pose covariance is inflated by (15 cm, 15 cm, 30°) and the update retried.
- **Other details:** the update uses the Joseph form. Each look takes the per-tag median of about 8 frames, needs at least 2 frames per tag, and ignores tags beyond 3 m.
- **Noise values:**
  - **R:** 1 cm + 2 cm/m for f and l; 3° + 5°/m for φ.
  - **Frame-to-frame jitter** measured in the noise test: 0.3 mm / 0.1 mm at 1.7 m. The real error is bias, ~4 cm at 1.7 m against the tape.
  - **Q:** drive 0.5 cm + 5%; heading drift 4°/m; turn 2° + TURN_STD_FRACTION·|a|.
- **Node parameters:** `turn_scale` and `drive_scale` make the robot command value/scale. `drift_deg_per_m` sets the filter's drift. `turn_gain` sets the filter's expected turn fraction, default `TURN_GAIN`.

## Runs (bundles in `runs/`, which is gitignored)
| Run | Bundle | Settings | Result |
|---|---|---|---|
| Noise test | `bundle_20261113_053954` | still on the square mark | Only tag 2 visible. Frame-to-frame jitter is sub-mm. |
| Square 1 | `bundle_20261113_054853` | v1 code | A turn came out 16° instead of 46°. Lost in loop 3. |
| Square 2 | `bundle_20261113_061255` | old code, scales 1.0 | 3 loops; ~26 cm against the mark, 3.9 cm best-fit. Its log folder in the bundle is from the wrong session. |
| **Square 3** | `bundle_20261113_062548` | old code (46° pieces), `TURN_SCALE=0.8 DRIVE_SCALE=0.92 DRIFT=6` | **Used for the report**, replayed with v2. 7 of 8 tags; tag 1 is never in view. |
| Octagon 1 | `bundle_20261113_064111` | old code, gain 0.75 | Placed on the square mark by mistake. Lost and left the arena. |
| Octagon 2 | `bundle_20261113_065225` | old code, gain 0.75 | Turns were 1.05× planned. At vertex 7 the heading innovation was −95°; lost, then Ctrl-C. |
| Octagon 3 | `bundle_20261113_212324` | v2, `TURN_SCALE=0.8` | Turns were 1.48× planned. Hit tag 2's wall in loop 2. |
| **Octagon 4** | `bundle_20261113_221216` | v2, `TURN_SCALE=1.0 DRIVE_SCALE=0.92 DRIFT=6` | **Used for the report.** 3 loops, 0 rejections. Turns were 1.23× planned against a filter gain of 0.9, so it was close to the limit. |

**Regenerating the report figures** (only if the filter or the start pose changes):
```
tools/.venv/bin/python tools/slam_replay.py report_hw3/square_run3/slam_20261113_062025.jsonl --start 0.762 0.762 0 --out report_hw3/square_run3/figures/square
tools/.venv/bin/python tools/slam_replay.py report_hw3/octagon_run4/slam_20261113_220705.jsonl --start 0.991 0.667 0 --out report_hw3/octagon_run4/figures/octagon
```

## Why the octagons failed (settled)
- **The turn gain changes between sessions.** Actual ÷ planned turn: 0.56× (squares), 1.05× (octagon 2), 1.48× (octagon 3, scale 0.8), 1.23× (octagon 4, scale 1.0). The likely cause is wheel slip or stalling at low turn power. The battery and the motor commands were ruled out.
- **Under-turning is harmless; over-turning isn't.** Each octagon vertex turns toward a new tag. If the robot over-turns, the mapped tag leaves the ±32° field of view before the new tag is added, so the new tag is placed with the wrong heading. The map rotates ~12° per vertex, the loop closure is rejected, and the robot drives off.
- **v2's fix:** 25° turn pieces, so a mapped tag stays in view and corrects the heading, plus a filter gain of 0.9 ± 35%.
- **Plots:** `report_hw3/diagnosis/failed_octagons_actual_vs_planned.png` (square 3 and octagons 2 and 3) and `report_hw3/octagon_run4/figures/octagon_actual_vs_planned.png`.

## v3 (later, branch `hw3-v3`, local only)
- **Learned turn gains:** kL and kR are added to the state (s = [x, y, θ, kL, kR, landmarks…]), with prior 1.0 ± 0.4 and a random walk of 0.03 per turn. In `predict_turn`, ∂θ/∂k = a.
- **Calibration wiggle:** `CAL_TURNS`, ±15° × 3 after the first look. The learned gain needs it: old logs, which have no wiggle, replay worse.
- **Stop when lost:** after `ABORT_FAILED_RECOVERIES = 2`, the follower raises `RobotLost`, stops the motors and logs `{'type': 'lost'}`.
- **Results so far:**
  - Sim: 0 recoveries for turn gains 0.56–1.48, median error 2–5 cm. Run it with `tools/slam_sim.py` (`--no-cal` turns off the wiggle).
  - Octagon run 4 replayed with v3: 10.4 cm against the X, 4.2 cm best-fit.
- **To test on the robot:** push the branch (ask the user first). Then on the robot:
  ```
  cd ~/ros2_ws/src/rubikpi_ros2 && git fetch && git switch hw3-v3
  cd ~/ros2_ws && colcon build --packages-select robot_control && source install/setup.bash
  TURN_SCALE=1.0 DRIVE_SCALE=0.92 DRIFT=6 ~/ros2_ws/src/rubikpi_ros2/hw3_tmux.sh octagon 3
  ```
  `git switch main` on the robot goes back to the report code.

## How the user works
- **Background:** a student, new to robotics. Wants step-by-step instructions, one step at a time, with the why. Give **exact robot commands**. Give positions in **cm and inches**.
- **Workflow:** edit on the Mac at `/Users/sohamlokhande/dev/CSE276/CSE276Robot` (no ROS installed there). Commit and push from the Mac; the robot pulls into `~/ros2_ws/src/rubikpi_ros2`.
- **End every message that changes robot code with the pull/build commands.** `--packages-select robot_control` is enough unless the apriltag config changes.
- **Robot:**
  - `ssh ubuntu@192.168.1.24`. The Mac is 192.168.1.12.
  - Foxglove at `ws://192.168.1.24:8765`, topics `/camera/image_raw/compressed` and `/detections`.
  - Harmless `systemd-journald` camera broadcasts appear in every tmux pane.
- **Calibration tool:** `ros2 run robot_control calibrate left|right|drive SECONDS`, with `motor_control` running.
- **Teammate:** Kumaran Karthik (`kumaran-3527`) writes the HW3 report. **Don't modify his `hw2` branch.**
- **Mac Python:** `tools/.venv/bin/python` (numpy and matplotlib).

## HW2 (done)
- **Status:** submitted. The final run is in `report_hw2/data/` and `runs/good_run_20261102_024506/`.
- **Report:** `report_hw2/report.tex`. The PDF `hw2_Soham_Lokhande_and_Kumaran_Karthik.pdf` is untracked in the repo root.
- **Zip:** `/Users/sohamlokhande/dev/CSE276/hw2_Soham_Lokhande_and_Kumaran_Karthik.zip`. Recipe:
  1. `git archive` the implementation folders.
  2. Strip comments and docstrings from our own Python files (check the AST is otherwise unchanged).
  3. Add the PDF, and exclude `__pycache__`.
- **Fixes carried into HW3:**
  - AprilTag reads compressed images (`image_transport: compressed`), giving 10 fps.
  - The turn calibration is the HW1 values: left 1.353 rad/s + 0.066 s, right 1.730 rad/s + 0.061 s.
- **Camera:** IMX219, run with `image_rectify:=true`; fx = 1012.8 at 1280 × 720.
- **Loose ends:** `CALIBRATION_PLAN.md` is from HW1 and out of date. `.DS_Store` is not in `.gitignore`.
