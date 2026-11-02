# HW2 report: sources

`report.tex` is the report. It needs `pdflatex` with `tikz`, `booktabs` and `hyperref`, which Overleaf has. Upload this whole folder, because `figures/` must sit next to the `.tex` file.

**Still to do in the `.tex`:**
- The YouTube link (`https://youtu.be/TODO` near the top).
- Optional: the tape measurements in part (h). A `% TODO` comment marks the spot.

## What's in here
| File | What it is |
|---|---|
| `data/run_20261102_024506.csv` | The final successful run, as logged by the follower: `t` (s since start), `event` (`move` every 50 ms while moving, `fix` after each look, `waypoint` when one is reached), `x, y` (m) and `theta` (rad) estimates, and `info` (the tags used in that fix). |
| `data/logs/follower.log` | The follower's console output, one block per stop: the motion it chose, which tags it saw (frames, used or rejected, range, measured pose), the fused pose and its std, the searches, and the final summary. |
| `data/logs/{camera,apriltag,motor}.log` | Output of the other three nodes for the same run. |
| `data/landmarks.txt`, `data/waypoints_hw2.txt` | The map and the route used in the run. |
| `data/tags_36h11.yaml` | AprilTag detector settings (tag size 0.1435 m, compressed images). |
| `data/camera_parameter.yaml` | Camera calibration (fx=1012.8, fy=1013.7, cx=629.1, cy=360.8 at 1280×720; RMS 0.58 px). |
| `figures/trajectory.png` | The part (i) plot: the estimate every 0.5 s, the fixes, the tags and the reference path. |

To re-make the plot, run this from the repo root:
```
tools/.venv/bin/python tools/plot_trajectory.py report_hw2/data/run_20261102_024506.csv
```
It writes the PNG next to the CSV and prints the totals and the waypoint estimates.

## Where each number in the report comes from
| Report part | Number | Source |
|---|---|---|
| (a) | Step 25 cm, waypoint reached within 5 cm, forward/back only within 15 cm, aim tolerance 5° | `robot_control/robot_control/closed_loop_follower.py` (constants at the top, `next_motion`) |
| (a) | Drive 0.263 m/s; turns 77.5°/s left, 99.1°/s right | `robot_control/robot_control/calibration.py` (HW1) |
| (c) | Measurement and motion noise, gates, lens offset 0.10 m | `robot_control/robot_control/localization.py` (constants at the top) |
| (c) | 1.02 m read at a taped 1.00 m | Camera check, see `HANDOFF.md` "Camera calibration" |
| (d) | Search at 0.59 m before (0,0): turn −32.4°, then +30.5° | `follower.log`, leg 3 (`search: turn -32.4 deg toward tag 0`) |
| (f) | 23 drives, 9 turns | `follower.log`: lines starting with `drive`, `turn`, `search: turn` and `face: turn` |
| (f) | 89.5 s total, 28.4 s moving | CSV: last `t`; moving time is the sum of the gaps between consecutive `move` rows that are under 0.2 s apart |
| (f) | Corrections average 2.8 cm / 0.9°, max 9.8 cm / 2.4° | CSV: each `fix` row with a tag, compared with the `move` row just before it |
| (g) | 5.73 m, 451° | `follower.log` last lines (`Driven 5.73 m, turned 451 deg, 89 s`); also the sum of the `drive` / `turn` amounts in the log |
| (g) | Legs 1.19 / 2.16 / 2.39 m | Sum of the `drive` lines between the `=== waypoint` lines |
| (g) | Reference 5.24 m and 360° | 1 + 2 + √5 m; turns 90 + 90 + 63.4 + 116.6° |
| (h) | Waypoint errors | `follower.log` `Done.` block, and the `waypoint` rows in the CSV |
| (i) | 30 of 34 stops with a tag fix | `follower.log`: 34 `pose (` lines, 4 of which say `no tag` |

## Other runs (not in the report)
The bundles of the failed runs are in `runs/` (gitignored, Mac only). Use them in (j) if you want to show the problems:
- `bundle_20261102_020733.tgz`: AprilTag received 1–3 images per 10 s, and the robot spun in place.
- `bundle_20261102_022048.tgz`: tag 2 had been placed behind (1,1) instead of (1,2), and every sighting of it was rejected ("position off by ~1.1 m").
- `bundle_20261102_023538.tgz`: tag 2 was still misplaced, and every sighting was rejected again. The new log line shows the tag at about (1.2, 1.4) instead of the map's (1.0, 2.4). This is how the placement mistake was found.
