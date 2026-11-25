# HW4 report prep: data and notes ready to write from

Everything below was produced from the two filmed runs (`./hw4_tmux.sh safety` and
`./hw4_tmux.sh time`), both successful, run back-to-back on 25 Nov 2026 from the same start
pose. This file is scratch/reference for writing the actual report, not the report itself.

## What HW4 is (for the report's intro/architecture section)
Known-map localization (not SLAM) reusing HW2's `closed_loop_follower.py` / `localization.py`
unchanged. The only new work is an offline path planner (`tools/planners.py`) that produces two
different paths through the same 8x8 ft arena, around one central virtual-obstacle square, for
the robot to drive with its existing closed-loop stop-and-go controller.

## Arena / obstacle / landmark map (`landmarks_hw4.txt`)
8 ft (2.438 m) square arena, origin at the bottom-left corner, +x along the bottom edge, +y up
the left edge (same frame as HW3's arena).

**Tags 0-7** (the arena boundary, used for localization only): clustered in pairs near each of
the 4 corners, each 12 in (0.3048 m) in from its corner along the wall.

| Tag | Wall | Position (m) | Faces |
|---|---|---|---|
| 0 | bottom | (0.3048, 0.0000) | 90° (+y) |
| 1 | bottom | (2.1336, 0.0000) | 90° (+y) |
| 2 | right | (2.4384, 0.3048) | 180° (−x) |
| 3 | right | (2.4384, 2.1336) | 180° (−x) |
| 4 | top | (2.1336, 2.4384) | 270° (−y) |
| 5 | top | (0.3048, 2.4384) | 270° (−y) |
| 6 | left | (0.0000, 2.1336) | 0° (+x) |
| 7 | left | (0.0000, 0.3048) | 0° (+x) |

**Tags 8-11** (no physical box; these 4 tags themselves mark the virtual obstacle): an 18 in
(0.4572 m) square centered at arena-center (1.2192, 1.2192) m, spanning 0.9906-1.4478 m on each
axis. Each tag sits at the midpoint of one side, facing straight outward along a cardinal
direction. The planner treats this square as blocked regardless of there being a real box.

| Tag | Side | Position (m) | Faces |
|---|---|---|---|
| 8 | east | (1.4478, 1.2192) | 0° (+x) |
| 9 | north | (1.2192, 1.4478) | 90° (+y) |
| 10 | west | (0.9906, 1.2192) | 180° (−x) |
| 11 | south | (1.2192, 0.9906) | 270° (−y) |

Start pose for both runs: (0.4064, 0.4064, 45°) = 16 in, 16 in from the bottom-left corner,
facing diagonally into the arena. Goal pose for both runs: (2.032, 2.032, 45°) = 80 in, 80 in.

## The two planners (`tools/planners.py`) — algorithm section material
Both share one world model: the obstacle rectangle inflated by the robot's radius (0.13 m) so a
path grazing the inflated boundary is collision-free for the real footprint; the arena walls are
inflated the same way.

- **Max-safety (grid representation).** Rasterize the arena into square cells; a cell is blocked
  if within `robot_radius` of the obstacle or a wall. Multi-source BFS from blocked cells gives
  every free cell's clearance (distance transform) to the nearest obstacle/wall. Dijkstra on the
  8-connected grid with edge cost `step_length / (clearance + eps)` is pulled toward the widest
  open space — it hugs the middle of the free corridor rather than skimming the obstacle's edge.
  A grid + clearance field is the natural representation when the objective *is* a
  distance-to-obstacle safety margin.
- **Min-distance/time (visibility-graph representation).** Nodes are the start, the goal, and the
  4 corners of the inflated obstacle rectangle. Two nodes connect if the straight segment between
  them doesn't cross the inflated rectangle. Dijkstra over this small graph gives the exact
  shortest piecewise-linear path (no grid discretization error) — what "minimum distance/time"
  calls for.

Generated with:
```
tools/.venv/bin/python tools/planners.py --start 0.4064 0.4064 45 --goal 2.032 2.032 45
```

**Planned waypoints** (`x, y, theta_rad`, consumed as-is by `waypoint_follower.load_waypoints`):

- `waypoints_hw4_safety.txt`: (0.4064, 0.4064, 0.7854) → (1.0050, 1.9350, 0.0942) → (2.0320,
  2.0320, 0.7854) — routes around the obstacle's far (upper-left) side, through open space.
- `waypoints_hw4_time.txt`: (0.4064, 0.4064, 0.7854) → (1.5778, 0.8606, 1.2009) → (2.0320, 2.0320,
  0.7854) — routes tight around the obstacle's near (lower-right) corner, the shorter path.

**Planner figures** (planned path over the clearance field / visibility graph):
`report_hw4/figures/safety_plan.png`, `report_hw4/figures/time_plan.png`.

## Robot runs: results

Both runs used `closed_loop_follower` v4 unchanged from HW2, launched via `hw4_tmux.sh`, 0
rejections, steady 10 fps camera/apriltag throughout (confirmed in both follower.logs). Both
reached the goal.

| | Safety run | Time run |
|---|---|---|
| Log bundle | `runs/bundle_safety_20261125_033650.tgz` | `runs/bundle_20261125_034145.tgz` |
| Run CSV | `run_20261125_033706.csv` | `run_20261125_034007.csv` |
| Waypoint 1 estimate | (0.980, 1.935, +7.4°), error 2.5 cm / −2.0° vs. planned (1.005, 1.935, 5.4°) | (1.578, 0.861, +68.8°), error 0.0 cm / +0.0° vs. planned (1.578, 0.861, 68.8°) |
| Waypoint 2 (goal) estimate | (2.014, 2.070, +45.0°), error 4.3 cm / +0.0° | (2.015, 2.024, +45.0°), error 1.9 cm / +0.0° |
| Distance driven | 2.88 m | **2.63 m** |
| Total turning | 233° | 255° |
| Wall-clock time | 53 s | 53 s |
| Tags used for fixes | 11, 3, 4, 5 | 11, 2, 3, 4 |
| Search sweeps needed (no tag at a stop) | 1 | 3 |

**Note on time vs. distance (a real, reportable finding, not a run failure):** the min-distance
plan did drive a shorter path (2.63 m vs. 2.88 m, matching the planner's objective), but it did
**not** finish faster — both took 53 s. The time run's tighter route past the obstacle's corner
produced worse tag-view geometry at a couple of stops, triggering 3 search sweeps (vs. 1 for
safety), and each search/turn/look cycle costs fixed time independent of path length. Under this
stop-and-go controller, per-stop sensing/search overhead dominates total time, not raw distance —
worth a sentence in the report's discussion/trade-offs section.

**Figures (all in `report_hw4/figures/`):**
- `safety_plan.png`, `time_plan.png` — planned path over the planner's internal representation.
- `safety_run_trajectory.png`, `time_run_trajectory.png` — actual driven trajectory (one dot per
  0.5 s of estimate, camera fixes circled) over the waypoints and the full landmark map, with
  driven distance/rotation/time in the title. Generated with:
  ```
  tools/.venv/bin/python tools/plot_trajectory.py <run>.csv --waypoints waypoints_hw4_<mode>.txt --landmarks landmarks_hw4.txt
  ```

## What's still needed before the report is complete
1. **Measure the true stop positions with a tape** for both runs (waypoint 2 in particular) and
   record them, to report actual error against ground truth rather than only the filter's own
   self-estimate. The 2.5/4.3 cm (safety) and 0.0/1.9 cm (time) errors above are *estimated*
   errors (filter's own belief at the moment it declared "reached"), not measured ones.
2. **Video**: film the robot driving both paths (not yet captured — the two logged runs above
   were run but check whether they were filmed; if not, re-run once more while filming, same
   start pose, since the setup already works reliably now).
3. **Report document itself**: not started. Suggested sections, reusing the assignment's ask:
   - Architecture: known-map localization reused from HW2, the two planner representations above.
   - Both algorithms with motivation (grid+clearance vs. visibility graph) — see planner section
     above, it's close to report-ready prose already.
   - The 4 figures above with captions.
   - Results table (above) plus measured (tape) error once available.
   - Discussion: the distance-vs-time trade-off finding above.
4. **Code zip**: same recipe as HW2/HW3 (`git archive` the implementation folders + `tools/`,
   strip comments/docstrings from own Python files, add the final PDF, exclude `__pycache__`).
