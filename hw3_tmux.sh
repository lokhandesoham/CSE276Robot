#!/bin/bash
# Start the HW3 nodes in a tmux session "hw3" (run on the robot):
#   camera | apriltag
#   slam_follower (typed, press Enter to start) | motor_control
#   ./hw3_tmux.sh square [loops]     square path (default 3 loops)
#   ./hw3_tmux.sh octagon [loops]    octagon path
#   TURN_SCALE=0.85 DRIVE_SCALE=0.9 DRIFT=6 ./hw3_tmux.sh square 3   with motion corrections
#   ./hw3_tmux.sh noise [seconds]    robot stands still, logs every tag frame (default 30 s)
# Logs: ~/hw3_runs/logs/<start time>/*.log and ~/hw3_runs/{slam,noise}_<time>.jsonl;
# ./hw3_bundle.sh packs the latest ones. `tmux kill-session -t hw3` stops all.
MODE=${1:-square}
ROOT=~/ros2_ws/src/rubikpi_ros2
case $MODE in
    square|octagon) ARGS="-p path_file:=$ROOT/path_hw3_$MODE.txt -p loops:=${2:-3}"
                    ARGS="$ARGS -p turn_scale:=${TURN_SCALE:-1.0} -p drive_scale:=${DRIVE_SCALE:-1.0} -p drift_deg_per_m:=${DRIFT:-0.0}" ;;
    noise) ARGS="-p stationary_time:=${2:-30.0}" ;;
    *) echo "usage: $0 square|octagon|noise [loops|seconds]"; exit 1 ;;
esac
LOGS=~/hw3_runs/logs/$(date +%Y%m%d_%H%M%S)_$MODE
mkdir -p "$LOGS"
S="source ~/ros2_ws/install/setup.bash && export PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0"
log() { echo "2>&1 | tee $LOGS/$1.log"; }

tmux kill-session -t hw3 2>/dev/null
tmux kill-session -t hw2 2>/dev/null
tmux new-session -d -s hw3 -n hw3
tmux send-keys -t hw3 "$S && ros2 launch robot_vision_camera robot_vision_camera.launch.py image_rectify:=true $(log camera)" C-m
tmux split-window -h -t hw3
tmux send-keys -t hw3 "$S && sleep 3 && ros2 launch apriltag_ros apriltag_launch.py $(log apriltag)" C-m
tmux split-window -v -t hw3
tmux send-keys -t hw3 "$S && ros2 run robot_control motor_control $(log motor)" C-m
tmux select-pane -L -t hw3
tmux split-window -v -t hw3
tmux send-keys -t hw3 "$S && ros2 run robot_control slam_follower --ros-args $ARGS $(log follower)"
echo "Logging to $LOGS"
if [ -n "$TMUX" ]; then
    tmux switch-client -t hw3
else
    tmux attach -t hw3
fi
