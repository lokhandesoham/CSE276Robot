"""HW4 path planners: max-safety (grid + distance transform + weighted Dijkstra) and
min-distance/time (inflated-obstacle visibility graph + Dijkstra).

Usage (on the Mac, from the repo root, no ROS needed):
    tools/.venv/bin/python tools/planners.py \\
        --landmarks landmarks_hw4.txt \\
        --obstacle 0.9906 0.9906 1.4478 1.4478 --robot-radius 0.13 \\
        --start 0.4064 0.4064 45 --goal 2.032 2.032 45 \\
        --out-dir .

Writes waypoints_hw4_safety.txt / waypoints_hw4_time.txt (consumed as-is by
robot_control.waypoint_follower.load_waypoints / closed_loop_follower) and
report_hw4/figures/{safety,time}_plan.png.

Both planners share one world model: an axis-aligned rectangle obstacle, inflated by the
robot's radius so a path that just grazes the inflated boundary is collision-free for the
real robot footprint, inside the arena square (whose walls are inflated the same way).

Safety planner (grid representation):
    Rasterize the arena into square cells. A cell is blocked if it is within robot_radius of
    the obstacle or of a wall. A multi-source BFS from the blocked cells gives every free
    cell's clearance (in cells) to the nearest obstacle/wall. Dijkstra on the 8-connected grid
    with edge cost = step_length / (clearance + eps) is pulled toward the widest open space,
    i.e. it hugs the middle of the free corridor around the obstacle rather than its edge.
    A grid + clearance field is the natural representation when the objective to optimize
    *is* a distance-based safety margin.

Min-distance/time planner (visibility-graph representation):
    Nodes are the start, the goal, and the 4 corners of the inflated obstacle rectangle. Two
    nodes are connected if the straight segment between them does not cross the inflated
    rectangle. Dijkstra over this small graph gives the exact shortest piecewise-linear path
    (no grid discretization error), which is what "minimum distance/time" calls for.
"""
import argparse
import heapq
import math
import pathlib
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / 'robot_control'))
from robot_control.localization import load_landmarks  # noqa: E402


ARENA_SIZE = 2.438          # m, 8 ft
# 1.5 ft (18 in = 0.4572 m) box, centered at arena-center (4 ft, 4 ft) = (1.2192, 1.2192) m,
# i.e. 39 in to 57 in on each axis -- within the assignment's 1x1-2x2 ft range.
DEFAULT_OBSTACLE = (0.9906, 0.9906, 1.4478, 1.4478)   # xmin, ymin, xmax, ymax (raw, not inflated)
DEFAULT_ROBOT_RADIUS = 0.13
DEFAULT_CELL = 0.03         # m, grid resolution for the safety planner
EPS_CLEARANCE = 1.0         # cells, keeps the safety cost finite right next to the boundary


# ---------------------------------------------------------------------------------- geometry
def inflate(rect, radius):
    xmin, ymin, xmax, ymax = rect
    return (xmin - radius, ymin - radius, xmax + radius, ymax + radius)


def point_in_rect(x, y, rect):
    xmin, ymin, xmax, ymax = rect
    return xmin <= x <= xmax and ymin <= y <= ymax


def segment_clear(p0, p1, rect, samples=60, shrink=1e-4):
    """True if the segment p0->p1 never enters the rectangle's interior.

    `shrink` pulls the rectangle in by a hair so a segment that only grazes its boundary
    (e.g. between two of its own corners, which the visibility graph needs) still counts
    as clear; it does not meaningfully change the robot's collision margin.
    """
    xmin, ymin, xmax, ymax = rect
    inner = (xmin + shrink, ymin + shrink, xmax - shrink, ymax - shrink)
    for t in np.linspace(0.0, 1.0, samples):
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        if point_in_rect(x, y, inner):
            return False
    return True


# ---------------------------------------------------------------------------------- safety: grid
class Grid:
    def __init__(self, arena_size, obstacle, robot_radius, cell):
        self.cell = cell
        self.n = int(math.ceil(arena_size / cell))
        self.arena_size = arena_size
        inflated = inflate(obstacle, robot_radius)
        blocked = np.zeros((self.n, self.n), dtype=bool)
        xs = (np.arange(self.n) + 0.5) * cell
        ys = (np.arange(self.n) + 0.5) * cell
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        blocked |= (X >= inflated[0]) & (X <= inflated[2]) & (Y >= inflated[1]) & (Y <= inflated[3])
        blocked |= (X < robot_radius) | (X > arena_size - robot_radius)
        blocked |= (Y < robot_radius) | (Y > arena_size - robot_radius)
        self.blocked = blocked
        self.xs, self.ys = xs, ys
        self.clearance = self._distance_transform()

    def _distance_transform(self):
        """Multi-source BFS from every blocked cell: clearance in cells for each free cell"""
        INF = np.inf
        dist = np.full((self.n, self.n), INF)
        from collections import deque
        q = deque()
        blocked_idx = np.argwhere(self.blocked)
        for i, j in blocked_idx:
            dist[i, j] = 0.0
            q.append((i, j))
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while q:
            i, j = q.popleft()
            for di, dj in neighbors:
                ni, nj = i + di, j + dj
                if 0 <= ni < self.n and 0 <= nj < self.n:
                    step = math.hypot(di, dj)
                    nd = dist[i, j] + step
                    if nd < dist[ni, nj]:
                        dist[ni, nj] = nd
                        q.append((ni, nj))
        return dist

    def cell_of(self, x, y):
        i = int(np.clip(round(x / self.cell - 0.5), 0, self.n - 1))
        j = int(np.clip(round(y / self.cell - 0.5), 0, self.n - 1))
        return i, j

    def world_of(self, i, j):
        return self.xs[i], self.ys[j]


def plan_safety(grid, start_xy, goal_xy):
    """Dijkstra on the grid, edge cost = step_length / (clearance + eps). Returns a world polyline."""
    s = grid.cell_of(*start_xy)
    g = grid.cell_of(*goal_xy)
    if grid.blocked[s] or grid.blocked[g]:
        raise SystemExit(f'start {start_xy} or goal {goal_xy} falls in a blocked cell; '
                          'check --robot-radius / --obstacle / arena size')

    n = grid.n
    dist = np.full((n, n), np.inf)
    dist[s] = 0.0
    prev = {}
    visited = np.zeros((n, n), dtype=bool)
    pq = [(0.0, s)]
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while pq:
        d, (i, j) = heapq.heappop(pq)
        if visited[i, j]:
            continue
        visited[i, j] = True
        if (i, j) == g:
            break
        for di, dj in neighbors:
            ni, nj = i + di, j + dj
            if not (0 <= ni < n and 0 <= nj < n) or grid.blocked[ni, nj] or visited[ni, nj]:
                continue
            step = math.hypot(di, dj) * grid.cell
            clearance = 0.5 * (grid.clearance[i, j] + grid.clearance[ni, nj])
            cost = step / (clearance + EPS_CLEARANCE)
            nd = d + cost
            if nd < dist[ni, nj]:
                dist[ni, nj] = nd
                prev[(ni, nj)] = (i, j)
                heapq.heappush(pq, (nd, (ni, nj)))

    if not visited[g]:
        raise SystemExit('safety planner: no free path found between start and goal')

    cells = [g]
    while cells[-1] != s:
        cells.append(prev[cells[-1]])
    cells.reverse()
    world = [grid.world_of(*c) for c in cells]
    world[0], world[-1] = start_xy, goal_xy
    return simplify(world, grid.blocked_rect_check())


def simplify(points, is_blocked_segment):
    """Greedy line-of-sight reduction: keep the farthest point each straight hop can reach"""
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and is_blocked_segment(points[i], points[j]):
            j -= 1
        out.append(points[j])
        i = j
    return out


# ---------------------------------------------------------------------------------- min-distance: visibility graph
def plan_shortest(obstacle, robot_radius, arena_size, start_xy, goal_xy):
    inflated = inflate(obstacle, robot_radius)
    xmin, ymin, xmax, ymax = inflated
    corners = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
    nodes = [start_xy, goal_xy] + corners
    n = len(nodes)

    def clear(a, b):
        if not segment_clear(a, b, inflated):
            return False
        margin = robot_radius - 1e-6
        for t in np.linspace(0.0, 1.0, 40):
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            if x < margin or x > arena_size - margin or y < margin or y > arena_size - margin:
                return False
        return True

    edges = {i: [] for i in range(n)}
    for i in range(n):
        for j in range(i + 1, n):
            if clear(nodes[i], nodes[j]):
                d = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                edges[i].append((j, d))
                edges[j].append((i, d))

    dist = [math.inf] * n
    dist[0] = 0.0
    prev = [None] * n
    visited = [False] * n
    pq = [(0.0, 0)]
    while pq:
        d, u = heapq.heappop(pq)
        if visited[u]:
            continue
        visited[u] = True
        if u == 1:
            break
        for v, w in edges[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if dist[1] == math.inf:
        raise SystemExit('time planner: start and goal are not connected in the visibility graph '
                          '(obstacle too close to a wall?)')

    path_idx = [1]
    while path_idx[-1] != 0:
        path_idx.append(prev[path_idx[-1]])
    path_idx.reverse()
    return [nodes[i] for i in path_idx], nodes, edges


# ---------------------------------------------------------------------------------- waypoints / figures
def to_waypoints(points, goal_theta_rad):
    waypoints = []
    for i, (x, y) in enumerate(points):
        if i < len(points) - 1:
            nx, ny = points[i + 1]
            theta = math.atan2(ny - y, nx - x)
        else:
            theta = goal_theta_rad
        waypoints.append((x, y, theta))
    return waypoints


def write_waypoints(path, waypoints, description):
    """Pure 'x, y, theta' lines, no '#' header: waypoint_follower.load_waypoints (used by
    closed_loop_follower) does not strip comments, unlike ekf_slam.load_path."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        for x, y, theta in waypoints:
            f.write(f'{x:.4f}, {y:.4f}, {theta:.4f}\n')
    print(f'wrote {path} ({description})')


def draw_landmarks(ax, landmarks):
    for tag_id, (x, y, facing) in sorted(landmarks.items()):
        ax.plot([x - 0.07 * math.sin(facing), x + 0.07 * math.sin(facing)],
                [y + 0.07 * math.cos(facing), y - 0.07 * math.cos(facing)], color='tab:blue', lw=4)
        ax.annotate(f'{tag_id}', (x, y), textcoords='offset points', xytext=(4, 4), fontsize=7)


def draw_start_goal(ax, start_xy, goal_xy):
    ax.scatter(*start_xy, marker='*', s=200, color='tab:green', zorder=5, label='start')
    ax.scatter(*goal_xy, marker='*', s=200, color='tab:red', zorder=5, label='goal')


def base_axes(arena_size, obstacle):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-0.1, arena_size + 0.1)
    ax.set_ylim(-0.1, arena_size + 0.1)
    ax.set_aspect('equal')
    ax.add_patch(plt.Rectangle((0, 0), arena_size, arena_size, fill=False, color='black', lw=1.5))
    xmin, ymin, xmax, ymax = obstacle
    ax.add_patch(plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                                facecolor='0.3', edgecolor='black', zorder=2, label='obstacle'))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    return fig, ax


def plot_safety(grid, obstacle, landmarks, start_xy, goal_xy, waypoints, out_path):
    fig, ax = base_axes(grid.arena_size, obstacle)
    clearance_m = np.where(grid.blocked, np.nan, grid.clearance * grid.cell)
    im = ax.imshow(clearance_m.T, origin='lower', extent=(0, grid.arena_size, 0, grid.arena_size),
                    cmap='viridis', alpha=0.6, zorder=0)
    fig.colorbar(im, ax=ax, label='clearance (m)', shrink=0.8)
    draw_landmarks(ax, landmarks)
    xs = [w[0] for w in waypoints]
    ys = [w[1] for w in waypoints]
    ax.plot(xs, ys, '-o', color='tab:orange', lw=2, ms=4, zorder=4, label='planned path')
    draw_start_goal(ax, start_xy, goal_xy)
    ax.set_title('Max-safety plan: grid + clearance field + weighted Dijkstra')
    ax.legend(loc='upper left', fontsize=8)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'wrote {out_path}')


def plot_time(obstacle, nodes, edges, landmarks, start_xy, goal_xy, waypoints, out_path):
    fig, ax = base_axes(ARENA_SIZE, obstacle)
    for i, neighbors in edges.items():
        for j, _ in neighbors:
            if j > i:
                ax.plot([nodes[i][0], nodes[j][0]], [nodes[i][1], nodes[j][1]],
                        color='0.75', lw=0.8, zorder=1)
    ax.scatter([n[0] for n in nodes], [n[1] for n in nodes], color='0.5', s=20, zorder=2,
               label='visibility-graph nodes')
    draw_landmarks(ax, landmarks)
    xs = [w[0] for w in waypoints]
    ys = [w[1] for w in waypoints]
    ax.plot(xs, ys, '-o', color='tab:orange', lw=2, ms=4, zorder=4, label='planned path')
    draw_start_goal(ax, start_xy, goal_xy)
    ax.set_title('Min-distance/time plan: inflated-obstacle visibility graph + Dijkstra')
    ax.legend(loc='upper left', fontsize=8)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'wrote {out_path}')


# patch Grid with a helper that plan_safety's simplify() needs (segment-vs-occupancy check)
def _grid_blocked_rect_check(self):
    def blocked(a, b, samples=40):
        for t in np.linspace(0.0, 1.0, samples):
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            i, j = self.cell_of(x, y)
            if self.blocked[i, j]:
                return True
        return False
    return blocked


Grid.blocked_rect_check = _grid_blocked_rect_check


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--landmarks', default=REPO / 'landmarks_hw4.txt')
    parser.add_argument('--obstacle', type=float, nargs=4, default=DEFAULT_OBSTACLE,
                         metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'))
    parser.add_argument('--robot-radius', type=float, default=DEFAULT_ROBOT_RADIUS)
    parser.add_argument('--arena-size', type=float, default=ARENA_SIZE)
    parser.add_argument('--cell', type=float, default=DEFAULT_CELL)
    parser.add_argument('--start', type=float, nargs=3, required=True, metavar=('X', 'Y', 'THETA_DEG'))
    parser.add_argument('--goal', type=float, nargs=3, required=True, metavar=('X', 'Y', 'THETA_DEG'))
    parser.add_argument('--out-dir', type=pathlib.Path, default=REPO)
    parser.add_argument('--fig-dir', type=pathlib.Path, default=REPO / 'report_hw4' / 'figures')
    args = parser.parse_args()

    landmarks = load_landmarks(args.landmarks)
    start_xy = (args.start[0], args.start[1])
    start_theta = math.radians(args.start[2])
    goal_xy = (args.goal[0], args.goal[1])
    goal_theta = math.radians(args.goal[2])

    grid = Grid(args.arena_size, tuple(args.obstacle), args.robot_radius, args.cell)
    safety_pts = plan_safety(grid, start_xy, goal_xy)
    safety_wp = to_waypoints(safety_pts, goal_theta)
    safety_wp[0] = (start_xy[0], start_xy[1], start_theta)
    write_waypoints(args.out_dir / 'waypoints_hw4_safety.txt', safety_wp,
                     'max-safety')
    plot_safety(grid, tuple(args.obstacle), landmarks, start_xy, goal_xy, safety_wp,
                args.fig_dir / 'safety_plan.png')

    time_pts, nodes, edges = plan_shortest(tuple(args.obstacle), args.robot_radius,
                                           args.arena_size, start_xy, goal_xy)
    time_wp = to_waypoints(time_pts, goal_theta)
    time_wp[0] = (start_xy[0], start_xy[1], start_theta)
    write_waypoints(args.out_dir / 'waypoints_hw4_time.txt', time_wp,
                     'min-distance/time')
    plot_time(tuple(args.obstacle), nodes, edges, landmarks, start_xy, goal_xy, time_wp,
              args.fig_dir / 'time_plan.png')

    def path_length(wp):
        return sum(math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1])
                   for i in range(len(wp) - 1))

    print(f'safety path: {len(safety_wp)} waypoints, {path_length(safety_wp):.2f} m')
    print(f'time path:   {len(time_wp)} waypoints, {path_length(time_wp):.2f} m')


if __name__ == '__main__':
    main()
