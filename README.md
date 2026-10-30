# CSE 276A Robotics Workspace (Rubik Pi + ROS 2 Humble)

ROS 2 codebase for **CSE 276A — Introduction to Robotics** at UC San Diego on the skid-steer Rover / Qualcomm Rubik Pi platform:
- **Homework 1**: Calibrated open-loop `Turn–Drive–Turn` unicycle waypoint controller.
- **Homework 2 (+20% Extra Credit, No AprilTags, No SLAM)**: Closed-loop visual navigation using a single Ultralytics YOLO detector (`yolov8s-worldv2` or `yolov8s`), camera lens undistortion, Range–Bearing Extended Kalman Filter (EKF) localization against a known landmark map, and automated $0.5\text{ s}$ trajectory logging.

---

## Repository Structure

```text
CSE276Robot/
├── robot_control/
│   ├── config/
│   │   └── hw2_config.yaml            # HW2 waypoints, unit scale, landmark map, YOLO & EKF settings
│   ├── launch/
│   │   ├── hw2_launch.py              # Full HW2 closed-loop stack launch file
│   │   └── robot_teleop_launch.py     # Keyboard teleop launch file
│   └── robot_control/
│       ├── motor_control.py           # 20 Hz UART motor driver (/dev/ttyUSB0) + 0.15s safety stop
│       ├── calibration.py             # HW1 calibrated unicycle drive/turn rates and delays
│       ├── calibrate.py               # HW1 open-loop step-response calibration runner
│       ├── waypoint_follower.py       # HW1 open-loop waypoint follower
│       ├── calibrate_camera.py        # HW2 interactive chessboard camera calibrator (OpenCV YAML)
│       ├── landmark_detector.py       # HW2 YOLO-World / YOLOv8s detector + undistorted (r, alpha)
│       ├── pose_estimator.py          # HW2 Range-Bearing EKF fusing motion model + landmarks
│       ├── closed_loop_follower.py    # HW2 closed-loop waypoint controller + 0.5s trajectory logger
│       └── plot_hw2_results.py        # HW2 2D trajectory dot plot & LaTeX pgfplots generator
├── robot_vision/
│   ├── config/camera_parameter.yaml   # OpenCV FileStorage camera intrinsics (K) and distortion (D)
│   ├── launch/robot_vision_camera.launch.py
│   └── src/robot_vision_camera.cpp    # GStreamer CSI camera publisher + Foxglove bridge
└── report/
    ├── report-hw1.tex                 # HW1 LaTeX report
    └── report-hw2.tex                 # HW2 LaTeX report
```

---

## 1. Setup & Build on Rubik Pi

Install Python dependencies (once on the Rubik Pi) and build the ROS 2 workspace:

```bash
pip3 install ultralytics pyyaml pyserial

cd ~/ros2_ws/src/rubikpi_ros2   # or your workspace root
colcon build --packages-select robot_vision_camera robot_control
source install/setup.bash
```

---

## 2. Homework 2: Closed-Loop Visual Navigation

### Step 2.1: Chessboard Camera Calibration

`robot_vision_camera` requires `camera_parameter.yaml` in OpenCV `cv::FileStorage` (`%YAML:1.0` `!!opencv-matrix`) format. Use `calibrate_camera` with a printed chessboard:

1. **Terminal 1 — Start Camera & Foxglove Bridge**:
   ```bash
   ros2 launch robot_vision_camera robot_vision_camera.launch.py
   ```
2. **Terminal 2 — Run Chessboard Calibration**:
   ```bash
   # Default: 9x6 inner corners, 25mm (0.025m) square size, 20 samples
   ros2 run robot_control calibrate_camera --ros-args -p cols:=9 -p rows:=6 -p square_size:=0.025
   ```
   - Open **Foxglove Studio** (`ws://<rubik-pi-ip>:8765`) and view `/calibration_debug_image/compressed`.
   - Note: `cols` and `rows` count **inner intersections** (e.g., a 10x7 grid of squares has `9x6` inner corners). If your printout has a different inner-corner grid, `calibrate_camera` automatically detects common grid sizes on the first frame.
   - Slowly move and tilt the board across the field of view until 20 samples are captured (or press `Ctrl+C` after $\ge 8$ samples). The calibrated $K$ and $D$ matrices are automatically written to `robot_vision/config/camera_parameter.yaml`.

---

### Step 2.2: Course & Landmark Setup ($1\text{ unit} = 0.5\text{ m}$)

Place small floor markers at the 4 waypoints and position the 5 landmarks at their known world coordinates (`robot_control/config/hw2_config.yaml`):

#### Waypoints
| Waypoint | Coordinates (units) | Coordinates (meters, `unit_scale_m: 0.5`) | Goal Heading |
| :--- | :--- | :--- | :--- |
| **$W_0$ (Start)** | $(0, 0, 0)$ | $(0.00\text{ m},\ 0.00\text{ m})$ | $0\text{ rad}$ ($0^\circ$, facing $+X$) |
| **$W_1$** | $(1, 0, 0)$ | $(0.50\text{ m},\ 0.00\text{ m})$ | $0\text{ rad}$ ($0^\circ$, facing $+X$) |
| **$W_2$** | $(1, 2, \pi)$ | $(0.50\text{ m},\ 1.00\text{ m})$ | $\pi\text{ rad}$ ($180^\circ$, facing $-X$) |
| **$W_3$ (End)** | $(0, 0, 0)$ | $(0.00\text{ m},\ 0.00\text{ m})$ | $0\text{ rad}$ ($0^\circ$, facing $+X$) |

#### Default Landmarks (`hw2_config.yaml`)
| ID | Name | World Pos (units) | World Pos (meters) | Height `height_m` | YOLO-World Prompt | COCO Classes (`yolov8s`) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **0** | `coke_can` | $(2.0,\ 0.0)$ | $(1.00\text{ m},\ 0.00\text{ m})$ | $0.122\text{ m}$ | `"red coke can"` | `["cup", "bottle"]` |
| **1** | `backpack` | $(1.8,\ 1.0)$ | $(0.90\text{ m},\ 0.50\text{ m})$ | $0.420\text{ m}$ | `"backpack"` | `["backpack", "handbag", "suitcase"]` |
| **2** | `tv_monitor` | $(1.0,\ 3.0)$ | $(0.50\text{ m},\ 1.50\text{ m})$ | $0.320\text{ m}$ | `"tv monitor"` | `["tv", "laptop"]` |
| **3** | `sports_ball` | $(-0.6,\ 2.0)$ | $(-0.30\text{ m},\ 1.00\text{ m})$ | $0.220\text{ m}$ | `"sports ball"` | `["sports ball", "orange", "bowl"]` |
| **4** | `chair` | $(-0.6,\ -0.4)$ | $(-0.30\text{ m},\ -0.20\text{ m})$ | $0.800\text{ m}$ | `"chair"` | `["chair", "potted plant", "stop sign"]` |

> **Tip**: Measure the actual height (`height_m`) of the objects you use and update `robot_control/config/hw2_config.yaml`. If a landmark's estimated range `r=` in `/landmark_debug_image/compressed` is slightly off at a known $1.0\text{ m}$ distance, adjust its `scale_factor` in `hw2_config.yaml`.

---

### Step 2.3: Verify Detector & EKF Before Driving

Before running the motors, verify landmark detection and range-bearing estimation in Foxglove Studio:

```bash
# Terminal 1: Camera + Foxglove
ros2 launch robot_vision_camera robot_vision_camera.launch.py

# Terminal 2: Landmark detector (choose model_mode:=yolo_world or model_mode:=yolov8s)
ros2 run robot_control landmark_detector --ros-args -p model_mode:=yolo_world

# Terminal 3: EKF Pose Estimator
ros2 run robot_control pose_estimator
```
- In Foxglove, subscribe to `/landmark_debug_image/compressed` to see bounding boxes, confidence, range $r$ (meters), and bearing $\alpha$ (degrees).
- Echo `/robot_pose` with `ros2 topic echo /robot_pose` to verify `[x_m, y_m, theta_rad, std_x, std_y, std_theta, visible_landmarks, secs_since_vis]`.

---

### Step 2.4: Run Full Closed-Loop Navigation

With `robot_vision_camera` running in Terminal 1, launch the full closed-loop stack in Terminal 2:

```bash
# Option 1: Using YOLO-World (default)
ros2 launch robot_control hw2_launch.py model_mode:=yolo_world unit_scale_m:=0.5

# Option 2: Using standard YOLOv8s COCO classes
ros2 launch robot_control hw2_launch.py model_mode:=yolov8s unit_scale_m:=0.5

# Dry run (no motor motion, tests state machine & log generation)
ros2 launch robot_control hw2_launch.py dry_run:=true
```

At the end of the run, `closed_loop_follower` saves:
- `hw2_trajectory.csv`: Estimated robot pose sampled every $0.5\text{ s}$.
- `hw2_summary.json`: Total translation distance, total rotation, and localization errors at each waypoint ($W_0, W_1, W_2, W_3$).

---

### Step 2.5: Generate 2D Trajectory Plot & Report Data

Run `plot_hw2_results` to generate the 2D trajectory image (`hw2_trajectory_plot.png`) and print the LaTeX `pgfplots` coordinates for `report/report-hw2.tex`:

```bash
ros2 run robot_control plot_hw2_results
```

---

## 3. Homework 1 Reference Commands

```bash
# Terminal 1: Start motor serial driver
ros2 run robot_control motor_control

# Terminal 2: Run open-loop calibration test (drive / left / right for duration s)
ros2 run robot_control calibrate drive 2.5

# Terminal 2: Run HW1 open-loop waypoint follower
ros2 run robot_control waypoint_follower
```
