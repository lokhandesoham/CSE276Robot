# Handoff: CSE 276A HW2 (closed-loop waypoint driving with AprilTags)

Last updated 2026-11-12. `main` is at `c96d26e` and in sync with GitHub.

## Status
**HW2 is done.**
- The robot completed the route on 2026-11-02.
- The report is written and compiled.
- The submission zip is built.
- Nothing is in progress. Any new session starts from a new request.

| Deliverable | Where | State |
|---|---|---|
| Final run | `report_hw2/data/` (CSV + 4 node logs), also `runs/good_run_20261102_024506/` | Done |
| Report (LaTeX) | `report_hw2/report.tex`, figure in `report_hw2/figures/trajectory.png` | Done. The user compiled it on Overleaf. |
| Report (PDF) | `hw2_Soham_Lokhande_and_Kumaran_Karthik.pdf` in the repo root | Untracked (not committed) |
| Data map | `report_hw2/README.md`: every number in the report → its file/log line | Done |
| Submission zip | `/Users/sohamlokhande/dev/CSE276/hw2_Soham_Lokhande_and_Kumaran_Karthik.zip` (outside the repo) | Built, see "Submission zip" |
| Video | The user handles it. The report has a `https://youtu.be/TODO` placeholder unless the user replaced it. | User's job |
| Resume | A project entry for this work was drafted in chat (not saved anywhere) | Done |

## Final run (2026-11-02, `run_20261102_024506`)
| | |
|---|---|
| Waypoints reached | All 3 |
| Estimated error (wp1 / wp2 / wp3) | 9.6 cm 1.5° / 4.7 cm 0.0° / 3.5 cm 2.0° |
| Totals | 5.73 m driven, 451° turned, 89.5 s (28.4 s moving) |
| Stops | 34, of which 30 had a tag fix, 0 rejected |
| Camera corrections | Average 2.8 cm / 0.9°, largest 9.8 cm / 2.4° |

- **Source of the waypoint errors:** they are the robot's own estimates. The user chose to report them as the error and took no tape measurements; the report says so.
- **The 0.0° at wp2:** it's dead reckoning, because no tag is visible facing π at (1,2).

## What fixed the robot (this session, in order)
1. **Turn calibration back to the HW1 values** (`e341875`). The ~45° turns that prompted the doubled turn times were caused by a low battery. On a full charge, `calibration.py` uses left 1.353 rad/s + 0.066 s and right 1.730 rad/s + 0.061 s.
2. **AprilTag reads the compressed images** (`85f1857`).
   - The cause: only 1–3 of the 10 raw 2.7 MB frames per second reached `apriltag_node`. All 10 `camera_info` messages per second arrived, so the images were lost in transport.
   - The fix: `apriltag_ros/cfg/tags_36h11.yaml` now has `image_transport: compressed`. This needs `ros-jazzy-image-transport-plugins` on the robot, which is installed.
   - The result: 10 fps reaching AprilTag.
3. **Search turns measured from the current heading** (`85f1857`). Before this, every map-guided search turn was computed from the heading the search started at, so the second turn overshot by 67°.
4. **The tag-2 placement mistake.** The user had placed tag 2 behind (1,1) instead of (1,2). The outlier gate correctly rejected every sighting ("position off by ~1.1 m"). `628e177` added "tag seen at (x, y), map has (…)" to rejection messages, which showed tag 2 at about (1.2, 1.4).

## Known limitations (listed in the report's reflection, not fixed)
- **Sideways error near a waypoint.** Within 15 cm (`APPROACH_RANGE`) the follower only drives forward/back, so a sideways offset stays (8.8 cm at wp1). Fix idea: a turn-drive-turn manoeuvre at 15–30 cm.
- **Noisy tag heading at range.** Tag yaw (PnP) is noisy beyond ~2 m. Fix idea: derive heading from the bearing `atan2(l, f)` plus the known tag position.
- **No tag in view at (1,2) facing π.** Fix idea: a 4th tag near (0.4, 2.0) facing +x.
- **Lens offset not measured.** `CAMERA_OFFSET = 0.10` m in `localization.py` is a guess. The floor marks go under the turning centre.
- **Slow.** The stop-and-go design leaves the robot standing still 68% of the time.

## Submission zip
The folder `hw2_Soham_Lokhande_and_Kumaran_Karthik/` contains:
- the report PDF;
- `robot_control/`, `robot_vision/`, `apriltag_ros/`;
- `landmarks.txt` and `waypoints_hw2.txt`.

What the user asked for:
- **Implementation code only:** no scripts, tools, run data or notes.
- **Comments removed:** both `#` comments and docstrings, from the Python files we wrote: `closed_loop_follower.py`, `localization.py`, `calibration.py`, `calibrate.py`, `capture_frames.py`, `waypoint_follower.py`.
  - Also stripped: the comment lines in `landmarks.txt`, and the comments we had added to `tags_36h11.yaml`.
  - Each Python file was checked to parse to the same AST as the original with only the docstrings removed.
  - Course and upstream files keep their original comments.
- **The repo still has all its comments.** Only the zip copy is stripped.
- **To rebuild the zip:**
  1. Export with `git archive HEAD robot_control robot_vision apriltag_ros landmarks.txt waypoints_hw2.txt`.
  2. Strip the comments with a tokenize/AST script. It lived in the session scratchpad and is gone, so rewrite it.
  3. Add the PDF.
  4. Zip without `__pycache__`.

## Who the user is / how they work
- **Background:** student new to robotics. Wants step-by-step instructions, one step at a time, with the *why*.
- **Workflow:**
  - Edit on the Mac at `/Users/sohamlokhande/dev/CSE276/CSE276Robot`. ROS isn't installed on the Mac.
  - Commit and push from the Mac.
  - The user pulls and builds on the robot at `~/ros2_ws/src/rubikpi_ros2`.
- **End every message that changes robot code with:**
  ```bash
  cd ~/ros2_ws/src/rubikpi_ros2 && git pull
  cd ~/ros2_ws && colcon build --packages-select robot_control robot_vision_camera apriltag_ros --allow-overriding apriltag_ros && source install/setup.bash
  ```
  If only `robot_control` changed, `--packages-select robot_control` is enough.
- **Teammate:** Kumaran Karthik (`kumaran-3527`) is a co-author on the report. His `hw2` branch has an option-2 (YOLO landmarks) implementation. **Don't modify `hw2`.**

## Running the robot (if needed again)
- **Robot access:**
  - Rubik Pi 3, Ubuntu 24.04, ROS 2 Jazzy.
  - `ssh ubuntu@192.168.1.24`. The Mac is 192.168.1.12.
  - Fallback: the USB-C serial console (`screen /dev/cu.usbmodem… 115200`).
- **Start:** `~/ros2_ws/src/rubikpi_ros2/hw2_tmux.sh`.
  - It starts one tmux session `hw2` with 4 panes: camera | apriltag / follower | motor.
  - The follower command is typed but waits for Enter.
  - Check its first lines: `closed_loop_follower v4` and `Camera ~10 fps, apriltag ~10 fps`.
  - Stop everything with `tmux kill-session -t hw2`.
- **After a run:**
  1. On the robot, run `~/ros2_ws/src/rubikpi_ros2/hw2_bundle.sh`. It packs the logs and the run CSV into `~/hw2_runs/bundle_<time>.tgz` and prints an `scp` command.
  2. Copy the bundle to the Mac's `runs/` (gitignored).
  3. Debug from the bundle plus the user's description of what the robot did.
- **Plot a run:** `tools/.venv/bin/python tools/plot_trajectory.py RUN.csv` gives a PNG with a dot per 0.5 s, and prints the totals.
- **Battery:** charge it fully before runs. Turn rates roughly halve on a low battery.
- **Camera:**
  - The camera is an IMX219; the original OV5647 module is dead.
  - Run with `image_rectify:=true`.
  - Calibration: fx=1012.8, fy=1013.7, cx=629.1, cy=360.8 at 1280×720, RMS 0.58 px.
  - Tag size 0.1435 m (the printer scaled the tags to 94%).
  - If the camera misbehaves after restarts, kill leftover processes (`pgrep -af "robot_vision|apriltag|foxglove"`) or reboot.
- **Foxglove:** `ws://192.168.1.24:8765`. Use `/camera/image_raw/compressed`.

## How the follower works (`robot_control/robot_control/closed_loop_follower.py`, `localization.py`)
The follower drives stop-and-go: stop → settle 1.0 s → look 0.8 s → decide → move.

- **Look:**
  - Collects the per-tag median over the frames, and needs the tag in ≥ 2 frames.
  - Runs a scalar Kalman update, heading first, then position using the fused heading.
  - Measurement noise grows with range. A 3σ gate (at least 30° / 0.40 m) rejects outliers.
- **Decide** (`next_motion`):
  - Within 5 cm: the waypoint is reached.
  - Within 15 cm: drive forward/back only.
  - Otherwise: turn if more than 5° off, else drive 25 cm, or the whole remainder if it's ≤ 40 cm.
  - At a reached waypoint: turn to the waypoint heading (no tag needed).
- **Tag rule:** a drive happens only if the stop just before it used a tag. Otherwise it runs `search()`:
  1. Up to 2 map-predicted turns toward visible tags, with targets fixed at the search start and each turn measured from the current heading.
  2. Then 40° sweep steps around a full circle.
  3. If nothing is found, it raises an error.
- **Turns over 95°** are split into equal pieces of at most 90°.
- **Map** (`landmarks.txt`, columns `id, x, y, facing_deg`):
  - tag 0 at (−0.4, 0) facing 0°
  - tag 1 at (1.4, 0) facing 180°
  - tag 2 at (1.0, 2.4) facing −90°
  - Each tag stands 0.4 m past its waypoint.
- **Tag frame convention** (verified): the tag's z axis points out of the printed face.
  - Robot-frame sighting: `f = t_z + CAMERA_OFFSET`, `l = −t_x`, face direction `α = atan2(−n_x, n_z)`.
  - `θ = φ − α`, and `(x, y) = tag − Rot(θ)(f, l)`.
- **Output:**
  - `~/hw2_runs/run_<time>.csv`, with a pose every 50 ms while moving, at every fix and at every waypoint.
  - Per-pane logs in `~/hw2_runs/logs/<time>/`.

## Loose ends
- **The report PDF is untracked** in the repo root. Ask before committing it.
- **`CALIBRATION_PLAN.md` is out of date** (from HW1). The HW1 report is in `report/`.
- **`.DS_Store` files** are not in `.gitignore`.
