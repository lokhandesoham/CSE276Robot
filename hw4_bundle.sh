#!/bin/bash
# Pack the latest HW4 session logs (camera/apriltag/motor/follower panes, from hw4_tmux.sh) and
# the latest run CSV (written by closed_loop_follower into ~/hw2_runs/, unchanged from HW2) into
# one file: ~/hw4_runs/bundle_<time>.tgz
# Copy it to the Mac with:
#   scp ubuntu@<robot-ip>:~/hw4_runs/bundle_<time>.tgz ~/dev/CSE276/CSE276Robot/runs/
mkdir -p ~/hw4_runs
cd ~/hw4_runs || exit 1
LOGS=$(ls -d logs/*/ 2>/dev/null | sort | tail -1)
CSV=$(ls -t ~/hw2_runs/run_*.csv 2>/dev/null | head -1)
OUT=bundle_$(date +%Y%m%d_%H%M%S).tgz
tar czf "$OUT" $LOGS $CSV
echo "Packed: $LOGS $CSV"
echo "scp ubuntu@$(hostname -I | awk '{print $1}'):~/hw4_runs/$OUT ~/dev/CSE276/CSE276Robot/runs/"
