# Handoff: CSE 276A HW2, closed-loop waypoint driving with AprilTags

Last updated 2026-11-01. `main` at `f30974d`, in sync with GitHub. `HANDOFF.md` and `.DS_Store` are untracked.

## Status in one paragraph
The stop-and-go closed-loop follower is written, simulated on the Mac, and pushed. **No successful real run yet.** The latest real run (2026-11-01) drove 0.70 m, never got a tag fix, swept a full circle and stopped, because **the camera is delivering ~1 frame/s and AprilTag receives only 0-3 images per 10 s** (Open issue #1). Fix the camera first, then do a full run, debugging from the log bundle.

## The task (hw2-2026.pdf, due 2026-10-28)
- **Route** (`waypoints_hw2.txt`): (0,0,0) → (1,0,0) → (1,2,π) → (0,0,0), metres and radians, closed-loop with camera feedback from landmarks at known positions.
- **Option chosen:** option 1 (AprilTags). Option 2 (learned landmarks, +20%) postponed (see "Branches").
- **Deliverables:** ~3-page report (items a–j: controller graph, landmark placement, pose estimation and noise, no-landmark handling, smoothness, total distance incl. rotation, error at each waypoint, 2D trajectory with a dot every 0.5 s from the robot's own estimates, reflection), unlisted YouTube video with floor markers at the waypoints, the code.
- **The user's priority:** get the robot driving the route reliably first. The report comes later.

## Open issues (start here)
1. **Camera frame rate.** AprilTag's sync warnings showed ~10 CameraInfo per 10 s (the camera node publishes one per frame, so the camera makes ~1 fps instead of 10) and only 0-3 images per 10 s reaching AprilTag. Earlier sessions saw a steady 10 Hz. Asked the user to run, with the session up, `ros2 topic hz /camera/camera_info` and `ros2 topic hz /camera/image_raw` and report both. Next step depends on the answer:
   - camera_info ~1 Hz: the camera node itself is slow. Try `image_compress:=false` (skips a JPEG encode per frame in the GStreamer callback, `robot_vision_camera.cpp:436`) and close Foxglove; check `top`; reboot to reset `cam-server`.
   - camera_info ~10 Hz but image_raw low: 2.7 MB raw images dropped by DDS. Look at socket buffer sizes (`net.core.rmem_max`) / the RMW in use.
   The follower's look window is 0.8 s and needs a tag in ≥ 2 frames (`MIN_FRAMES`), so it needs ≥ ~5 fps. It now prints `Camera X fps, apriltag Y fps` at start and frame counts at every no-tag stop.
2. **Motion check pending.** The user asked for HW1-style checks before the next run (`calibrate drive 3.80` → 100 cm, `calibrate left 2.46` / `right 1.94` → 90°). Results not reported yet. If the drive is off, refit `DRIVE_SPEED`.
3. **Reference point conflict** (see "Who the user is"): the code assumes floor marks under the **turning centre** with the lens `CAMERA_OFFSET` = 0.10 m ahead (a guess, never measured). The earlier session recorded the user's decision that the lens is the reference with offset 0. Ask the user once: measure the offset, or set `CAMERA_OFFSET = 0` and place the lens on the marks (then turns in place move the lens ~offset off the mark).
4. **First real run behaviour** (before the latest fixes, older code): it reached (1,0) fine but did not turn 90° toward (1,2), and on another run kept driving without seeing tags. Causes found/fixed since: turn calibration (HW1 times gave ~45° on this floor), tiny-motion loops near waypoints, and drives allowed after a stop with no tag. Not yet confirmed on the robot. The follower prints `closed_loop_follower v4: ...` at start; if that line is missing the robot runs old code.

## How the follower works (`robot_control/robot_control/closed_loop_follower.py`, `localization.py`)
Stop → look → think → move, repeated; all motion is open-loop fixed commands (HW1 style), the loop closes at each stop.
- **Move:** `turn_command` / `drive_command` from `calibration.py`, published at 20 Hz for the computed time, then stop, then `settle_time` 1.0 s. Turns over 95° are split into equal pieces ≤ 90°, each with the settle pause.
- **Look** (`localize`): 0.8 s of `/tf` camera_frame→tag_N transforms, per-tag median over frames (≥ 2 frames needed), then a scalar Kalman update (heading first, then position using the fused heading). Measurement noise grows with range; prediction variance grows per motion; 3σ gates (minimum 30° / 0.40 m) reject outliers. Tag-frame convention as in "AprilTag frame convention" below.
- **Think** (`next_motion`): within 5 cm → reached. Within 15 cm → only drive the forward/back part (no turning). Else turn if > 5° off, else drive 25 cm (`step` parameter), or all the way if ≤ 40 cm left. Minimum motions 4 cm / 5°.
- **Tag rule:** a drive only happens if the stop right before it used a tag (or right after a successful search + turn back). Otherwise `search()`: turn toward the tags `tag_view_turns` (map-based, in localization.py) predicts are visible (up to 2), then sweep 40° steps for a full circle; nothing found → stop the run with an error.
- **Waypoint:** reached when the estimate is within 5 cm; then turn to the waypoint heading (no tag needed), look, re-approach once if pushed off. At (1,2) facing π no tag is visible, so that heading is dead reckoning.
- **Output:** `~/hw2_runs/run_<time>.csv` (pose every 50 ms while moving, every fix, every waypoint) and a summary with estimated waypoint errors and total distance/rotation. Mac: `tools/.venv/bin/python tools/plot_trajectory.py RUN.csv` → PNG with a dot per 0.5 s (report item i) plus totals.
- **Config files** (repo root, read from `~/ros2_ws/src/rubikpi_ros2/`): `waypoints_hw2.txt`, `landmarks.txt` (`id, x, y, facing_deg`).
- **Simulation** (Mac, no ROS): stub `rclpy`, `std_msgs`, `tf2_msgs`, `sensor_msgs`, `apriltag_msgs`, `rclpy.qos` on `PYTHONPATH`, subclass the node to override `wait` (fake clock + synthetic tag TFs within a 64° FOV) and `move` (apply noisy true motion; skip the outer call for split turns). Last results: all waypoints within ~3 cm with the calibrated model, ~6-9 cm with pessimistic stiction, 0 drives without a preceding tag. The sim scripts lived in the session scratchpad and are gone; rebuild if needed.

## Turn calibration change (2026-10-31)
On the HW2 floor the HW1 turn times turned only ~45°; the user tested doubled times (left 2.46 s, right 1.94 s) and got 90°. `calibration.py` now has the rates halved and delays doubled (left 0.6765 rad/s + 0.132 s, right 0.865 rad/s + 0.122 s); drive unchanged (0.263 m/s, 1 m = 3.80 s). The old 1.23 s → 45° also fits the new line.

## Who the user is / how they want to work
- Student new to robotics. Wants step-by-step instructions, one step at a time, stopping at checkpoints. Wants explanations of *why* (e.g. how calibration works without a known distance).
- **Workflow:**
  - Edit on the Mac at `/Users/sohamlokhande/dev/CSE276/CSE276Robot`. ROS isn't installed on the Mac.
  - Commit and push; the user has accepted this so far.
  - The user pulls and builds on the robot at `~/ros2_ws/src/rubikpi_ros2`, a clone of `github.com/lokhandesoham/CSE276Robot`.
- **End every message that changes code with the pull/rebuild reminder:**
  ```bash
  cd ~/ros2_ws/src/rubikpi_ros2 && git pull
  cd ~/ros2_ws && colcon build --packages-select robot_control robot_vision_camera apriltag_ros --allow-overriding apriltag_ros && source install/setup.bash
  ```
- **User decision (earlier session):** the **camera lens is the robot's reference point**, camera-to-robot translation 0. **Conflict, unresolved:** this session's code uses the *turning centre* as the reference (`CAMERA_OFFSET = 0.10` in `localization.py`, a guess) and told the user to put the turning centre on the floor marks, because turns in place move the lens off the mark. See "Open issues" #3.
- **User decision:** keep the **HW1 motor calibration** (`robot_control/calibration.py`) for all motions. Exception, by the user's own test on 2026-10-31: turn times doubled (see below).

## Robot access and running things
- **Robot:** Rubik Pi 3, Ubuntu 24.04, ROS 2 Jazzy, user `ubuntu`, IP **192.168.1.24** on home Wi-Fi. The Mac is 192.168.1.12.
- **Mac → robot access:** SSH (`ssh ubuntu@192.168.1.24`). Fallback: the USB-C debug serial port (`screen /dev/cu.usbmodem… 115200`; quit with Ctrl-A, K, Y; leftover sessions block the port, check with `screen -ls`).
- **Terminals:** the user runs nodes in **tmux**. Mistakes so far: pasting several long-running commands into one window; creating a new session instead of a window (Ctrl-B C); typos in launch file names.
- **Foxglove:**
  - Connect from app.foxglove.dev at `ws://192.168.1.24:8765`. The bridge is started by the camera launch, so it's down whenever the camera launch is stopped.
  - Use `/camera/image_raw/compressed` in Foxglove; raw images are about 27 MB/s.
  - Foxglove's lag is Wi-Fi only. On the robot, frames arrived at a steady 10.0 Hz (earlier session), and the nodes used about 1 of 8 cores. **On 2026-11-01 this regressed to ~1 Hz, see "Open issues" #1.**
- **Standard start-up:** `~/ros2_ws/src/rubikpi_ros2/hw2_tmux.sh` (one tmux session, 4 panes: camera | apriltag / follower | motor_control; the follower command is typed but waits for Enter). Pasting long multi-line blocks into the terminal failed once (the lines after `colcon build` were swallowed), hence the script.
- **Logs:** the script tees every pane to `~/hw2_runs/logs/<time>/{camera,apriltag,motor,follower}.log`. After a run, `~/ros2_ws/src/rubikpi_ros2/hw2_bundle.sh` packs the latest logs and run CSV into `~/hw2_runs/bundle_<time>.tgz` and prints the `scp` command to copy it to the Mac's `runs/` (gitignored). **Debug runs from the bundle + the user's description of what the robot did.**
- **Stale processes / cam-server:** if the camera misbehaves after restarts, `pgrep -af "robot_vision|apriltag|foxglove"`, kill leftovers, or reboot (resets `cam-server`). journald "Broadcast message" spam in the panes is cosmetic.
- **`image_rectify:=true` is required** for correct AprilTag distances.

## Camera saga (solved, don't redo)
- **Faulty camera:** the original Arducam OV5647 fisheye never answered on I2C on either port (`NACK`, `read id: 0x0`), even with the TA's driver files from `github.com/notu97/rubikpi_flash` (all verified identical with `cmp`).
- **Fix:** swapped in a **Raspberry Pi Camera Module 2 (IMX219)**, which the Rubik Pi 3 supports natively, on the same 15-pin-to-22-pin cable. `cam-server` then reported `Number of cameras: 1`. The OV5647 module itself is bad.
- **Check command:** `sudo journalctl -u cam-server -b --no-pager | grep "Number of cameras"`.
- **Driver files:** `/usr/lib/qcm6490/camera/` now has the TA's stock OV5647 files; a colleague had patched `cam1_ov5647.bin`, and that was reverted. The IMX219 files were already there.

## Camera calibration (done)
- **Tools (Mac side):**
  - `tools/make_printables.py` → `printables/apriltags.pdf` (tag36h11 IDs 0–2) and `printables/checkerboard.pdf` (8×6 inner corners, 25 mm).
  - `robot_control/capture_frames.py` (robot): press Enter to save `/camera/image_raw` as PPM in `~/calib_imgs`.
  - `tools/calibrate_camera.py` (Mac, run with `tools/.venv/bin/python`, OpenCV 4.x).
- **Result:** 31 shots, RMS 0.58 px, fx=1012.8, fy=1013.7, cx=629.1, cy=360.8, at 1280×720. That's a 64° horizontal / 38° vertical field of view, which matches the IMX219. k3 is fixed at 0, because a free k3 fitted noise from shots weighted toward the image centre.
- **Projection matrix:** `projection_matrix` = the alpha=0 `getOptimalNewCameraMatrix`, because the camera node rectifies with exactly that matrix. **apriltag_ros computes poses from P only, with no distortion** (`AprilTagNode.cpp:181`, `pose_estimation.cpp:52`).
- **Tag size:** the tags printed at about 94% scale (the 100 mm check line measured 94 mm). The black square measures 143–144 mm, so `apriltag_ros/cfg/tags_36h11.yaml` has `size: 0.1435` and tracks IDs `[0, 1, 2]`.
  - That node **silently ignores IDs not listed** (`AprilTagNode.cpp:219`).
- **Distance check:** a tag at a taped 1.00 m reads z = 1.02 m. Before the size fix it was about 6% long at every distance.

## AprilTag frame convention (verified from real readings)
- **What the node outputs:** `tf2_echo camera_frame tag_N` gives the tag pose in the camera's optical frame (x right, y down, z forward).
- **Orientation:** for a tag squarely facing the camera, R ≈ diag(1, −1, −1). So the **tag's z axis is the outward normal of the tag face, pointing toward the viewer**; tag x is the viewer's right, and tag y is up.
- **2D math planned for the localizer:** tag `N` sits at world (tx, ty) and faces direction φ, the world angle of its outward normal.
  - Robot-frame tag position: forward `f = t_z`, left `l = −t_x`.
  - Normal in the robot frame: `n_f = R[2,2]`, `n_l = −R[0,2]`, and `α = atan2(n_l, n_f)`. α ≈ π when the tag faces the robot.
  - Heading: `θ = φ − α`. Position: `(x, y) = (tx, ty) − Rot(θ)·(f, l)`.
  - Checked against the readings: the robot at (1,0) facing +x sees tag 1 (at (1.4,0), φ=π) as f=0.4, so θ=0 and the position is (1,0).
- **`tf2_echo` pitfalls:** it keeps printing the **last** transform after a tag leaves the view, so check timestamps. It only follows the one tag frame you name. The user mixed this up once.

## Arena (as placed by the user)
World frame: origin at the start, +x toward (1,0), +y to the left, θ counter-clockwise. Tags stand 40 cm past their waypoint, with their centre at about camera height.

| Tag | Position (m) | Faces (φ) | Seen during |
|---|---|---|---|
| 1 | (1.4, 0.0) | −x (π) | leg 1 → (1,0); the final turn to 0 at (0,0) (1.4 m ahead) |
| 2 | (1.0, 2.4) | −y (−π/2) | leg 2 → (1,2) |
| 0 | **user placed it at (−0.4, 0.0), facing +x (0)** | | leg 3 → (0,0) |

- **Tag 0's placement:** `landmarks.txt` has it as placed, (−0.4, 0) facing +x. Recommended: move it to **(−0.18, −0.36), facing (1,2)** (φ ≈ 63.4°), 0.4 m past (0,0) on the leg-3 line, and change its line in `landmarks.txt` to `0, -0.18, -0.36, 63.4`. As placed, it drops out of view in the last ~40 cm of leg 3, which costs one search turn per run (simulated). The user hasn't said whether they moved it.
- **Known blind spot:** at waypoint (1,2,π), no tag lies within the ±32° field of view facing π. That heading relies on dead reckoning until the robot turns toward (0,0). Option: add a 4th tag at about (0.3–0.6, 2.0), facing +x.
- **Not done:** the user skipped the per-tag placement check (they trust the calibration). Expected readings: at (0,0)→+x tag_1 z=1.40; at (1,0)→+x tag_1 z=0.40; at (1,0)→+y tag_2 z=2.40; at (1,2)→+y tag_2 z=0.40. The one valid reading was tag 0 at 0.39 m with the robot at (0,0) facing −x, which is correct.

## Branches
- **`main`:** our work. It has the AprilTag plan, printables, calibration tools and the camera YAML.
- **`hw2`:** pushed 2026-10-29 by teammate **kumaran-3527**. It's a full option-2 implementation:
  - YOLO object landmarks, a range-bearing EKF, a closed-loop follower and plotting.
  - It also strips the comments from `calibration.py`, uses 1 unit = 0.5 m, and its `calibrate_camera` writes P = K (wrong for AprilTags).
- **The user's decision:** AprilTags on `main` first; look at `hw2` later for the object-detection extra credit. **Don't modify `hw2`.** The robot is checked out on `main`.

## HW1 (finished, kept for reference)
Package `robot_control`:
```
waypoint_follower ──/motor_commands [L, R] @20 Hz──► motor_control ──serial──► motor board
```
- **`motor_control`:** clips commands to ±0.5, sends zeros if nothing arrives for 0.15 s, writes to `/dev/ttyUSB0`.
- **Calibration** (tape on the wheels, waypoint floor, full battery):
  - drive `[0.196, 0.204]`: 0.263 m/s, so 1 m = 3.80 s;
  - left `[-0.3, 0.3]`: 77.5°/s plus 0.066 s;
  - right `[0.3, -0.3]`: 99.1°/s plus 0.061 s.
- **Repeatability:** turns vary about ±5° from run to run; the IMU is ignored.
- **Report:** `report/report.html` → `report/report.pdf`; the zip is at `/Users/sohamlokhande/dev/CSE276/HW1_waypoint_navigation.zip`.
- **Full history:** git log and the earlier version of this file (`git log -p HANDOFF.md` doesn't apply, because the file was never committed).

## Loose ends
- `CALIBRATION_PLAN.md` is out of date (from HW1).
- Older plan file: `~/.claude/plans/read-handoff-md-and-continue-generic-canyon.md` (superseded by the stop-and-go design above).
- Pull/rebuild reminder for this session's changes only needs `robot_control`; use the full command above when camera/apriltag packages change.
