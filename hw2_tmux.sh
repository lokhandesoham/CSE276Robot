#!/bin/bash
# Start the HW2 nodes in a tmux session "hw2" (run on the robot):
#   camera | apriltag
#   follower (typed, press Enter to start) | motor_control
# Every pane is also saved to ~/hw2_runs/logs/<start time>/{camera,apriltag,motor,follower}.log;
# after a run, ./hw2_bundle.sh packs the latest logs and run CSV into one file.
# Ctrl-b + arrow keys moves between panes, Ctrl-b d detaches, `tmux kill-session -t hw2` stops all.
LOGS=~/hw2_runs/logs/$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOGS"
# unbuffered, uncoloured output so the log files are complete and readable
S="source ~/ros2_ws/install/setup.bash && export PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0"
log() { echo "2>&1 | tee $LOGS/$1.log"; }

tmux kill-session -t hw2 2>/dev/null
tmux new-session -d -s hw2 -n hw2
tmux send-keys -t hw2 "$S && ros2 launch robot_vision_camera robot_vision_camera.launch.py image_rectify:=true $(log camera)" C-m
tmux split-window -h -t hw2
tmux send-keys -t hw2 "$S && sleep 3 && ros2 launch apriltag_ros apriltag_launch.py $(log apriltag)" C-m
tmux split-window -v -t hw2
tmux send-keys -t hw2 "$S && ros2 run robot_control motor_control $(log motor)" C-m
tmux select-pane -L -t hw2
tmux split-window -v -t hw2
tmux send-keys -t hw2 "$S && ros2 run robot_control closed_loop_follower $(log follower)"
echo "Logging to $LOGS"
if [ -n "$TMUX" ]; then
    tmux switch-client -t hw2   # already inside tmux: switch instead of nesting
else
    tmux attach -t hw2
fi
