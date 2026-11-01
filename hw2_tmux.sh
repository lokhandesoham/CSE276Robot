#!/bin/bash
# Start the HW2 nodes in a tmux session "hw2" (run on the robot):
#   camera | apriltag
#   follower (typed, press Enter to start) | motor_control
# Ctrl-b + arrow keys moves between panes, Ctrl-b d detaches, `tmux kill-session -t hw2` stops all.
S='source ~/ros2_ws/install/setup.bash'
tmux kill-session -t hw2 2>/dev/null
tmux new-session -d -s hw2 -n hw2
tmux send-keys -t hw2 "$S && ros2 launch robot_vision_camera robot_vision_camera.launch.py image_rectify:=true" C-m
tmux split-window -h -t hw2
tmux send-keys -t hw2 "$S && sleep 3 && ros2 launch apriltag_ros apriltag_launch.py" C-m
tmux split-window -v -t hw2
tmux send-keys -t hw2 "$S && ros2 run robot_control motor_control" C-m
tmux select-pane -L -t hw2
tmux split-window -v -t hw2
tmux send-keys -t hw2 "$S && ros2 run robot_control closed_loop_follower"
if [ -n "$TMUX" ]; then
    tmux switch-client -t hw2   # already inside tmux: switch instead of nesting
else
    tmux attach -t hw2
fi
