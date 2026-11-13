# Handoff: CSE 276A HW3 (EKF-SLAM with AprilTags), in progress

Last updated during the night of 12→13 Nov 2026. The robot clock is UTC, so a run named `..._064933` was at 06:49 UTC, which is 22:49 Pacific on 12 Nov.

**Deadline: Friday 13 Nov 2026, before midnight.** The assignment is `hw3-2026.pdf` and the lecture decks are `lec8a-2026.pdf` (Kalman filter / EKF) and `lec8b-2026.pdf` (SLAM), all in the repo root and untracked.

HW2 is finished; see "HW2 (done)" at the end.

## Status at a glance
| Item | State |
|---|---|
| Arena and ground truth | Done. Tags were placed at the planned positions, and the user says those **are** the measured ground truth (`ground_truth_hw3.txt`). |
| SLAM code (v2) | Done and pushed. **Turn fix written and pushed:** 25° turn pieces, TURN_GAIN 0.9, TURN_STD_FRACTION 0.35. Needs an octagon run on the robot. |
| Square run, for the report | **Done: square run 3.** Saved in `report_hw3/square_run3/`. Figures regenerated with the new filter: 9.4 cm mean landmark error (5.5 cm best-fit), 0 recoveries. |
| Octagon run | **Failing.** The robot loses its heading during loop 1 and drives out of the arena. This is the current task. |
| Noise test | Done: `runs/bundle_20261113_053954/`. |
| Lens-to-axle distance | **Still not given by the user.** `CAMERA_OFFSET = 0.10` m is a guess. The replay can correct it afterwards with `--camera-offset`. |
| Report | **Kumaran (teammate) writes it** after all runs are done. Give him the data folders, figures and the method notes below. |
| Video | The user films the runs. The report needs an unlisted YouTube link. |
| Code zip | Not built yet. Same recipe as HW2 (see "HW2 (done)"), plus the HW3 files. |

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

## Code (all on `main`)
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

## Runs so far (bundles in `runs/`, which is gitignored)
| Run | Bundle | Settings | Result |
|---|---|---|---|
| Noise test | `bundle_20261113_053954` | still on the square mark | Only tag 2 visible (tag 3 sits at the edge of the ±32° field of view). Jitter is sub-mm. |
| Square 1 | `bundle_20261113_054853` | v1 code | A turn came out 16° instead of 46°. The filter rejected everything afterwards and was lost in loop 3. This led to v2. |
| Square 2 | `bundle_20261113_061255` | v2, scales 1.0 | 3 loops. Its log folder in the bundle is from the wrong session; that bundle-script bug is fixed. Replay gives ~26 cm with the measured start pose and 3.9 cm best-fit. |
| **Square 3** | `bundle_20261113_062548` | v2, `TURN_SCALE=0.8 DRIVE_SCALE=0.92 DRIFT=6` | **Used for the report.** Replay error after loops 1/2/3: **14.7 / 5.6 / 7.0 cm** with the measured start pose, 4.9 cm best-fit, 0 recoveries. 7 of 8 tags; tag 1 is never in view on this path. |
| Octagon 1 | `bundle_20261113_064111` | same as square 3, plus filter `TURN_GAIN` 0.75 | Placed on the **square** mark by mistake. Lost at vertex 6 and left the arena. |
| Octagon 2 | `bundle_20261113_065225` | same | Correct mark, but the start reading suggests ~15 cm placement error. **Same failure:** at vertex 7, tag 2 was rejected with a −95° heading innovation; then the robot was lost and drove out (Ctrl-C). Diagnosis was in progress. |

**Report data for square run 3:** `report_hw3/square_run3/` holds the log, the 4 pane logs, the figures (`square_map.png`, `square_landmarks.png`, `square_loops.png`, `square_summary.json`), `replay_output.txt`, `motion_fit_output.txt`, the ground truth and the path. It was replayed with the current committed filter (TURN_GAIN 0.75, turn std 0.25). **If the filter changes, regenerate these figures** with:
```
tools/.venv/bin/python tools/slam_replay.py report_hw3/square_run3/slam_20261113_062025.jsonl --start 0.762 0.762 0 --out report_hw3/square_run3/figures/square
```
Then check that the numbers are still about 7 cm.

## The problem being solved now: turns are erratic
- **Turn outcomes vary widely.** `tools/slam_measure_turns.py` and tag-derived poses show that a 46° command (`TURN_CMD = 0.3`, 0.66 s) produced anywhere from **12° to 42°** in the square runs. In the octagon runs it produced about the **full amount or more** (e.g. 50° for a planned 42°, 50.5° for a planned 46°).
- **A longer command didn't help.** In square run 3 the scale of 0.8 made commands 25% longer, and the turns did not get longer.
- **It's not the motor commands.** The motor log shows no safety stops mid-motion.
- **It's not the battery.** The user says it was fully charged, and a stationary calibration test on a taped "+" looked fine to them.
- **The cause is likely** wheel slip or stalling at the low turn power, varying with the spot on the floor.

**Why the octagon fails and the square survives.**
- The filter's fixed TURN_GAIN of 0.75 (fitted on the square runs) underestimates the octagon's fuller turns by ~12–20° per vertex.
- At every octagon vertex the robot turns toward a **new** tag. No mapped tag is in view to correct the heading, so each new landmark is placed with the wrong heading. The map stays self-consistent but rotates further at each vertex.
- When the robot sees tag 2 again (the loop closure), the heading error is ~100°. Every sighting is rejected. The recovery inflation of 30° is too small, so the robot drives on a wrong estimate and leaves the arena.

## Diagnosis of octagon run 2 (done)
- **Measured turn.** The only octagon turn with the same tag in view before and after came out 1.10× planned. The filter assumed 0.75×.
- **Replay with the right gain.** With a gain of ~1.09 (`--kt 1.45`), octagon run 2 replays with 0 rejections and a 5.9 cm best-fit error. So the failure is the turn model, not the camera or the start pose.
- **No single gain fits.** Square runs need ~0.75 and fail at ≥ 0.95; the octagon needs ≥ 1.0. A grid over gain × std on all 5 logs found nothing that works for both.
- **Simulation, 6 seeds, ±30% erratic turns.**
  - 46° pieces with gain 0.75 and turns at 1.1× planned: the robot leaves the arena in 3–4 of 6 runs. This reproduces the real failure.
  - 25° pieces with gain 0.9 and std 0.35: 0 recoveries in both regimes (0.75× and 1.1× planned), median error 2–9 cm.
  - **Chosen:** 25° pieces, gain 0.9, std 0.35. Steps 1, 2, 4 and 5 below are done; 6 onwards remain.
- **Replaying octagon logs.** Their headers log `turn_gain` 0.75, so the replay uses that. Add `--kt 1.2` to replay them with the new model.

## Next step
1. **Smaller turn pieces.** Change `MAX_TURN_STEP` in `ekf_slam.py` from 46° to about **25°**. A tag that's already mapped then stays inside the ±32° field of view across each piece and corrects the heading before a new tag is added. This is the main fix.
2. **Turn model for erratic turns.** Set `TURN_GAIN` to about 0.85–0.9 and `TURN_STD_FRACTION` to about 0.35, instead of 0.75 / 0.25.
3. **Optionally,** raise the recovery heading inflation, e.g. to 60°, or add a re-anchoring step.
4. **Check in simulation** with erratic turns, comparing piece size 46° vs 25° over several seeds for both shapes:
   ```
   tools/.venv/bin/python tools/slam_sim.py octagon --loops 3 --seed N --turn-bias 0 --turn-spread 0.4 --out /tmp/x.jsonl
   tools/.venv/bin/python tools/slam_replay.py /tmp/x.jsonl
   ```
   A grid script for this was started and the user interrupted it; it is not saved. Make sure `ekf_slam.py` ends with the chosen values; the interrupted run once left `TURN_STD_FRACTION` changed, and it was reverted.
5. **Replay all real logs** (square 1–3, octagon 1–2) with the new settings. Square run 3 must stay at about 7 cm, and octagon run 2's loop 1 should no longer show the 95° jump. Note that replays can't change where the robot physically drove.
6. **Commit and push.** The user then pulls and builds, and reruns the octagon from the **octagon mark**, with the axle centre on the X and facing +x:
   ```
   cd ~/ros2_ws/src/rubikpi_ros2 && git pull
   cd ~/ros2_ws && colcon build --packages-select robot_control && source install/setup.bash
   TURN_SCALE=0.8 DRIVE_SCALE=0.92 DRIFT=6 ~/ros2_ws/src/rubikpi_ros2/hw3_tmux.sh octagon 3
   # after the run:
   tmux kill-session -t hw3; ~/ros2_ws/src/rubikpi_ros2/hw3_bundle.sh   # then scp to the Mac's runs/
   ```
   Consider dropping TURN_SCALE back to 1.0 when the filter's gain changes. The command scale and the filter gain should tell one consistent story.

   Check the first follower lines: `slam_follower v2 … filter turn gain …` and `apriltag ~10 fps`. The first look should show tag 2 at about **f = 1.45, l = +0.15** from the octagon mark.
7. **Analyse the good octagon run:** run `slam_replay.py` with `--start 0.991 0.667 0`, or the measured start pose if it differs. Save it to `report_hw3/octagon_runN/` the same way as the square. If the filter changed, also regenerate the square figures.
8. **Hand-off pack for Kumaran**, in `report_hw3/README.md`: arena sketch data, the EKF design above, the noise and motion-fit method, per-loop error tables for both shapes, the figures, why n = 3 ft and m = 1.5 ft (same 3.66 m path length; keeps tags 0.6–2 m away, inside the ~2.8 m reliable range and the ±32° field of view), and limitations (erratic turns, tag 1 unseen on the square, lens offset unmeasured, start-pose placement error rotating the map).
9. **Zip:** implementation code only. In HW2 the user wanted comments stripped in the zip copy only.

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
