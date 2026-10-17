# Fresh Calibration Plan (HW1)

Same floor and conditions as before. Old numbers are used **only to size the tests**. All values are measured again.

> 📘 The reasoning behind every step, with worked examples, is in [CALIBRATION_EXPLAINED.md](CALIBRATION_EXPLAINED.md).

## What the old data tells us (and how it shapes these tests)

| Old finding | What we do now |
|---|---|
| `0.2` drifted least (1 cm / 3 s). `0.3`–`0.4` drifted 8–11 cm right | Drive straight at **0.2** only |
| Old fit used only **2 points** for forward speed | Use **4 durations × 3 repeats** and fit a line with an intercept (start-up loss) |
| Waypoints need **1.00 m** and **1.41 m** drives, but with no localisation each run should stay short to limit sideways drift | Test **short** durations: 1.5, 2.0, 2.5, 3.0 s (about 26–70 cm). Nothing shorter than 1.5 s, because the robot is still speeding up |
| Turning at `0.5` = ~288°/s, so one 50 ms command period ≈ **14°**. The left fit was only within ±9° | Try a **slower turn command (0.3)** so one period is only a few degrees |
| Turn deadband was `0.15` | `0.3` is 2× the deadband, so it should turn reliably. The probe below checks this |
| Left and right turns differed (delay on the left) | Calibrate **left and right separately** |
| Waypoint turns: **45°, 90°, 135°, 180°** | Turn tests aim at exactly those angles |
| A 0.5 s pause after driving made the next turn come up short | Validation step checks the settle time |

## Setup (each session)

```bash
# Terminal 1 (on the robot)
cd ~/ros2_ws && colcon build --packages-select robot_control && source install/setup.bash
ros2 run robot_control motor_control

# Terminal 2
source ~/ros2_ws/install/setup.bash
```

- Start with a **full battery**. Write the time down. Re-run test F0 at the end to catch battery drop.
- Leave ~1 m of clear floor in front of the robot (the longest calibration run is ~70 cm).
- **Put the robot back on the taped start pose before every run.** Each run then starts from a known pose, so errors never carry over from one run to the next.
- Each `calibrate` run counts down 3 s, publishes at 20 Hz, then stops. The robot always starts from rest.

### How to measure

- **Distance:** before the run, tape a mark at the front-left wheel contact point. After the run, measure:
  - **forward distance** along the start line
  - **lateral drift** (+ = right)
- **Angle (two-line method, ±1°):**
  1. Before the turn, tape **line A** along the robot's side using a straight edge. After the turn, tape **line B** the same way.
  2. Extend both lines until they cross at a point O.
  3. Mark points **50 cm** from O along each line and measure the distance `c` between them.
  - `θ° = 114.6·asin(c/100)` with `c` in cm. This works wherever the robot pivots.
  - For turns near 180°, compare B with A **reversed** and use θ = 180° − leftover. See tutorial §7.

---

## 📋 Master run sheet (fill this in)

Run the rows **top to bottom**. Every command starts with `ros2 run robot_control calibrate`, shortened below to `cal`.
- **Before every run:** put the robot back on the taped start pose.
- **Units:** **d** = forward distance (cm), **dr** = sideways drift (cm, + = right), **θ** = angle (°), measured with the two-line method.
- **F1–F4:** do them in the order F1 F2 F3 F4, then the same again twice more. This fills the Run 1, Run 2 and Run 3 columns in turn.
- **Turn durations (rows 12–19):** these assume the probe gives about 120°/s. If your probe rate R is different, use `t = angle / R`, separately for left and right.

| # | ID | Command | Record | Run 1 | Run 2 | Run 3 | Why this test |
|---|---|---|---|---|---|---|---|
| 1 | F0-start | `cal 0.2 0.2 2.0` | d, dr, battery % / time | | — | — | Battery baseline. Compared with row 31 to see if the battery dropped during the session |
| 2 | F1 | `cal 0.2 0.2 1.5` | d, dr | | | | Shortest point on the speed line (~26 cm). Anchors `t_loss`. Not shorter, because the robot is still speeding up |
| 3 | F2 | `cal 0.2 0.2 2.0` | d, dr | | | | 2nd point on the speed line (~41 cm) |
| 4 | F3 | `cal 0.2 0.2 2.5` | d, dr | | | | 3rd point (~55 cm). Shows whether distance vs time is really a straight line |
| 5 | F4 | `cal 0.2 0.2 3.0` | d, dr | | | | Longest point (~70 cm). Largest drift, so it's the best place to judge whether a trim is needed |
| 6 | Trim-A *(only if F4 dr > 2 cm right)* | `cal 0.195 0.2 3.0` | d, dr | | | | Slows the left side slightly to cancel the rightward curve |
| 7 | Trim-B *(only if Trim-A still curves right)* | `cal 0.19 0.2 3.0` | d, dr | | | | Bigger trim. Keep whichever of rows 5–7 drives straightest |
| 8 | P1 | `cal -0.3 0.3 0.5` | θ, smooth or jerky? | | — | — | Checks that 0.3 turns left smoothly (2× the old 0.15 deadband) |
| 9 | P2 | `cal -0.3 0.3 1.0` | θ | | — | — | With P1: left rate R = (P2−P1)/0.5 °/s, used to size rows 12–15 |
| 10 | P3 | `cal 0.3 -0.3 0.5` | θ, smooth or jerky? | | — | — | Same check for right turns. Left and right differed before |
| 11 | P4 | `cal 0.3 -0.3 1.0` | θ | | — | — | With P3: right rate R, used to size rows 16–19 |
| 12 | L45 | `cal -0.3 0.3 0.38` | θ | | | | Short-angle point for the left fit. Shows whether left turns have a start-up delay (the old data did) |
| 13 | L90 | `cal -0.3 0.3 0.75` | θ | | | | Left fit point at 90°, the most common course turn |
| 14 | L135 | `cal -0.3 0.3 1.13` | θ | | | | Left fit point at 135° (final turn at WP6) |
| 15 | L180 | `cal -0.3 0.3 1.50` | θ | | | | Longest left turn. Pins down the left turn rate and gives a margin above 135° |
| 16 | R45 | `cal 0.3 -0.3 0.38` | θ | | | | Right fit point at 45° (WP5) |
| 17 | R90 | `cal 0.3 -0.3 0.75` | θ | | | | Right fit point at 90° (WP3, WP6) |
| 18 | R135 | `cal 0.3 -0.3 1.13` | θ | | | | Right fit point at 135°. Gives the fit a middle point |
| 19 | R180 | `cal 0.3 -0.3 1.50` | θ | | | | Right 180°, used on the course at WP4. The biggest turn, so the biggest error risk |
| — | **FIT** | *(no command)* | `DRIVE_SPEED`, `DRIVE_START_LOSS`, `TURN_RATE_L/R`, `TURN_DELAY_L/R` | | | | Fit lines to rows 2–5 and 12–19 (see Phases 1 & 3), then work out the durations for rows 20–26 |
| 20 | V1 | `cal 0.2 0.2 «t»` | d (target 50) | | — | — | Checks the drive fit on a distance it wasn't fitted to |
| 21 | V2 | `cal 0.2 0.2 «t»` | d (target 70) | | — | — | Checks the drive fit at the top of the tested range |
| 22 | V3 | `cal -0.3 0.3 «t»` | θ (target 90) | | — | — | Checks the left-turn fit at the most common angle |
| 23 | V4 | `cal 0.3 -0.3 «t»` | θ (target 90) | | — | — | Checks the right-turn fit at 90° |
| 24 | V5 | `cal 0.3 -0.3 «t»` | θ (target 180) | | — | — | Checks the course's hardest turn (WP4) |
| 25 | V6 | `cal -0.3 0.3 «t»` | θ (target 135) | | — | — | Checks the final left turn (WP6) |
| 26 | V7 *(optional)* | `cal 0.2 0.2 «t»` | d (target 100), dr | | — | — | The only long run. Checks that the short-run fit still holds at the course's 1 m legs |
| 27 | S1 | follower, `settle_time:=0.3` *(see Phase 4)* | θ of the turn after the drive | | — | — | Turn started right after a drive: does it come up short? |
| 28 | S2 | follower, `settle_time:=1.5` | θ of the turn after the drive | | — | — | Same with a 1.5 s pause. If S1 < S2, keep `settle_time` ≥ 1.5 |
| 29 | D1 *(optional)* | `cal 0.08 0.08 2.0` | d | | — | — | Forward deadband for the report. Step by ±0.01 until it just moves |
| 30 | D2 *(optional)* | `cal -0.15 0.15 2.0` | θ | | — | — | Turn deadband for the report. Shows how much margin 0.3 has |
| 31 | F0-end | `cal 0.2 0.2 2.0` | d, dr, battery % / time | | — | — | Battery drift. If d is >5% below row 1, recharge and recheck before the final run |

**Acceptance:** drives within ±3 cm/m, turns within ±5°. If a validation row misses, add it to the data, refit, and recompute.

---

## Phase 1 — Forward speed at 0.2

Model: `d = v · (t − t_loss)`

| ID | Command | Expected (old model) |
|---|---|---|
| F0 | `ros2 run robot_control calibrate 0.2 0.2 2.0` | ~41 cm (reference / battery check) |
| F1 | `ros2 run robot_control calibrate 0.2 0.2 1.5` | ~26 cm |
| F2 | `ros2 run robot_control calibrate 0.2 0.2 2.0` | ~41 cm |
| F3 | `ros2 run robot_control calibrate 0.2 0.2 2.5` | ~55 cm |
| F4 | `ros2 run robot_control calibrate 0.2 0.2 3.0` | ~70 cm |

Run **F1–F4 three times each** (12 runs), mixing the order: F1 F2 F3 F4, F1 F2 F3 F4, F1 F2 F3 F4. Run F0 once at the start and once at the end.

> Why not shorter? Below ~1.5 s most of the run is the robot speeding up, so the straight-line model `d = v·(t − t_loss)` stops fitting. Short runs also make the fixed timing error (±1.5 cm from the 50 ms tick) a bigger share of the distance. The 3 repeats average that out.

| ID | t (s) | Run 1 d / drift (cm) | Run 2 d / drift | Run 3 d / drift | Mean d |
|---|---|---|---|---|---|
| F1 | 1.5 | | | | |
| F2 | 2.0 | | | | |
| F3 | 2.5 | | | | |
| F4 | 3.0 | | | | |
| F0 start / end | 2.0 | | | — | |

**Fit:** least-squares line through all 12 points, `d = a·t + b`. Then:
- `DRIVE_SPEED = a`
- `DRIVE_START_LOSS = −b / a`

**Drift trim (only if mean drift > 2 cm at 3.0 s, i.e. ~3 cm/m):**
- Drift to the right means the left side is faster, so lower L slightly.
- Try `calibrate 0.19 0.2 3.0` and `calibrate 0.195 0.2 3.0`, and keep whichever is straightest.
- If you add a trim, redo F4 with it.

---

## Phase 2 — Choose the turn command (probe)

| ID | Command | Purpose |
|---|---|---|
| P1 | `ros2 run robot_control calibrate -0.3 0.3 0.5` | left, short |
| P2 | `ros2 run robot_control calibrate -0.3 0.3 1.0` | left, long |
| P3 | `ros2 run robot_control calibrate 0.3 -0.3 0.5` | right, short |
| P4 | `ros2 run robot_control calibrate 0.3 -0.3 1.0` | right, long |

Rough rate = `(θ_1.0 − θ_0.5) / 0.5` °/s, done separately for left and right.

Decide which turn command to use:

| Probe result | Turn command to use |
|---|---|
| ~80–200°/s, smooth, angles roughly double from 0.5 s → 1.0 s | **0.3** |
| Jerky / stalls / < 60°/s | Try **0.35**, then **0.4** (repeat P1–P4) |
| > 200°/s | Fine, but resolution is worse. Consider **0.25** |

Write down the chosen value as `TURN_CMD`. All of Phase 3 uses it.

---

## Phase 3 — Turn rate, left and right

Model: `θ = ω · (t − t_delay)`, fitted **separately** for left and right.

Pick durations that should give the waypoint angles, using the probe rate `R` (°/s):

`t = angle / R` (the fit will absorb any delay)

| Target | Duration formula | Example if R = 120°/s |
|---|---|---|
| 45° | 45 / R | 0.38 s |
| 90° | 90 / R | 0.75 s |
| 135° | 135 / R | 1.13 s |
| 180° | 180 / R | 1.50 s |

Commands (replace `T45` etc. with your computed durations and `0.3` with your `TURN_CMD`):

```bash
# LEFT (CCW, +)
ros2 run robot_control calibrate -0.3 0.3 T45
ros2 run robot_control calibrate -0.3 0.3 T90
ros2 run robot_control calibrate -0.3 0.3 T135
ros2 run robot_control calibrate -0.3 0.3 T180
# RIGHT (CW, −)
ros2 run robot_control calibrate 0.3 -0.3 T45
ros2 run robot_control calibrate 0.3 -0.3 T90
ros2 run robot_control calibrate 0.3 -0.3 T135
ros2 run robot_control calibrate 0.3 -0.3 T180
```

**3 repeats each** (24 runs). If you're short on time, do 3× at 90° and 180°, and 2× at 45° and 135°.

| Dir | Target | t (s) | Run 1 (°) | Run 2 (°) | Run 3 (°) | Mean (°) | Spread (°) |
|---|---|---|---|---|---|---|---|
| L | 45 | | | | | | |
| L | 90 | | | | | | |
| L | 135 | | | | | | |
| L | 180 | | | | | | |
| R | 45 | | | | | | |
| R | 90 | | | | | | |
| R | 135 | | | | | | |
| R | 180 | | | | | | |

**Fit (per direction):** `θ = a·t + b` gives `TURN_RATE = a` (convert to rad/s: × π/180) and `TURN_DELAY = −b / a`.
- If `TURN_DELAY` is under ~0.02 s or negative, fit through zero instead.
- The **spread** column tells you how repeatable the turns are. Compare it with the old ±9°.

---

## Phase 4 — Validation (use the new constants)

Work out each duration from your fits, then check the robot hits the target:
- drive: `t = d / DRIVE_SPEED + DRIVE_START_LOSS`
- turn: `t = θ / TURN_RATE + TURN_DELAY`

| ID | Motion | Command | Target | Measured |
|---|---|---|---|---|
| V1 | drive 0.50 m | `calibrate 0.2 0.2 «t»` | 50 cm | |
| V2 | drive 0.70 m | `calibrate 0.2 0.2 «t»` | 70 cm | |
| V3 | left 90° | `calibrate -TURN TURN «t»` | 90° | |
| V4 | right 90° | `calibrate TURN -TURN «t»` | 90° | |
| V5 | right 180° | `calibrate TURN -TURN «t»` | 180° | |
| V6 | left 135° | `calibrate -TURN TURN «t»` | 135° | |
| V7 *(optional, once)* | drive 1.00 m | `calibrate 0.2 0.2 «t»` | 100 cm | |

Aim for **±3 cm/m** on drives and **±5°** on turns. If a check misses, add that run to the fit and refit.

> **V7 is the only long run.** The course needs 1.00 m and 1.41 m drives, but the fit only covers up to ~70 cm, so predicting those distances goes beyond the tested range. One 1 m check shows whether that holds up. If V7 is off by more than ~3 cm, add it to the Phase 1 data and refit.

**Settle check (rows 27–28):** `calibrate` always counts down 3 s, so it can't start a turn straight after a drive. Use the waypoint follower with a 2-waypoint file instead: it drives 50 cm, then turns +90°. Do this **after** putting the new constants into `calibration.py` and rebuilding.

```bash
printf "0,0,0\n0.5,0,1.57\n" > ~/settle_test.txt
ros2 run robot_control waypoint_follower --ros-args -p waypoints_file:=$HOME/settle_test.txt -p settle_time:=0.3   # S1
ros2 run robot_control waypoint_follower --ros-args -p waypoints_file:=$HOME/settle_test.txt -p settle_time:=1.5   # S2
```

Measure the final heading. If S1 comes up short compared with S2, keep `settle_time ≥ 1.5`.

---

## Phase 5 — Optional extras
- **Deadband** (for the report only, not used by the follower): `calibrate 0.08 0.08 2.0` and `calibrate -0.15 0.15 2.0`. Step the value down or up by 0.01 until the robot just moves.
- **Battery effect:** compare F0 at the start and at the end. If they differ by > 5%, recalibrate on a fresh charge before the final run.

## Applying the results
Update these in `calibration.py`:
- `DRIVE_CMD`, `DRIVE_SPEED`, `DRIVE_START_LOSS`
- `TURN_CMD`, `TURN_RATE_LEFT`, `TURN_DELAY_LEFT`, `TURN_RATE_RIGHT`
- a `TURN_DELAY_RIGHT` if the right fit has a delay. This also needs a one-line change in `turn_command()`.

Then rebuild and dry-run the waypoint follower.
