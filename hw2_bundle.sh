#!/bin/bash
# Pack the latest HW2 session logs (all four panes) and the latest run CSV into one file:
#   ~/hw2_runs/bundle_<time>.tgz
# Copy it to the Mac with:
#   scp ubuntu@<robot-ip>:~/hw2_runs/bundle_<time>.tgz ~/dev/CSE276/CSE276Robot/runs/
cd ~/hw2_runs || exit 1
LOGS=$(ls -d logs/*/ 2>/dev/null | sort | tail -1)
CSV=$(ls run_*.csv 2>/dev/null | sort | tail -1)
OUT=bundle_$(date +%Y%m%d_%H%M%S).tgz
tar czf "$OUT" $LOGS $CSV
echo "Packed: $LOGS $CSV"
echo "scp ubuntu@$(hostname -I | awk '{print $1}'):~/hw2_runs/$OUT ~/dev/CSE276/CSE276Robot/runs/"
