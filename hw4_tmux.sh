#!/bin/bash
# Start the HW4 nodes in a tmux session "hw4" (run on the robot):
#   camera | apriltag
#   closed_loop_follower, driving one of the two planned paths (typed, press Enter) | motor_control
#   ./hw4_tmux.sh safety   max-safety path (waypoints_hw4_safety.txt)
#   ./hw4_tmux.sh time     min-distance/time path (waypoints_hw4_time.txt)
# Every pane is also saved to ~/hw4_runs/logs/<start time>_<mode>/{camera,apriltag,motor,follower}.log;
# the follower's own CSV goes to ~/hw2_runs/run_<time>.csv (plot with tools/plot_trajectory.py
# --waypoints waypoints_hw4_<mode>.txt --landmarks landmarks_hw4.txt).
# Ctrl-b + arrow keys moves between panes, Ctrl-b d detaches, `tmux kill-session -t hw4` stops all.
MODE=${1:?usage: $0 safety|time}
ROOT=~/ros2_ws/src/rubikpi_ros2
case $MODE in
    safety|time) ;;
    *) echo "usage: $0 safety|time"; exit 1 ;;
esac
ARGS="-p waypoints_file:=$ROOT/waypoints_hw4_$MODE.txt -p landmarks_file:=$ROOT/landmarks_hw4.txt"
LOGS=~/hw4_runs/logs/$(date +%Y%m%d_%H%M%S)_$MODE
mkdir -p "$LOGS"
S="source ~/ros2_ws/install/setup.bash && export PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0"
log() { echo "2>&1 | tee $LOGS/$1.log"; }

tmux kill-session -t hw4 2>/dev/null
tmux new-session -d -s hw4 -n hw4
tmux send-keys -t hw4 "$S && ros2 launch robot_vision_camera robot_vision_camera.launch.py image_rectify:=true $(log camera)" C-m
tmux split-window -h -t hw4
tmux send-keys -t hw4 "$S && sleep 3 && ros2 launch apriltag_ros apriltag_launch.py $(log apriltag)" C-m
tmux split-window -v -t hw4
tmux send-keys -t hw4 "$S && ros2 run robot_control motor_control $(log motor)" C-m
tmux select-pane -L -t hw4
tmux split-window -v -t hw4
tmux send-keys -t hw4 "$S && ros2 run robot_control closed_loop_follower --ros-args $ARGS $(log follower)"
echo "Logging to $LOGS"
if [ -n "$TMUX" ]; then
    tmux switch-client -t hw4
else
    tmux attach -t hw4
fi
