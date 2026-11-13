#!/bin/bash
# Pack the latest HW3 session logs and the latest .jsonl run into ~/hw3_runs/bundle_<time>.tgz
cd ~/hw3_runs || exit 1
# the newest run file, and the newest log folder that has a follower log (a session that was
# started but not run would otherwise be picked)
RUN=$(ls -t slam_*.jsonl noise_*.jsonl 2>/dev/null | head -1)
LOGS=$(dirname "$(ls -t logs/*/follower.log 2>/dev/null | head -1)")/
OUT=bundle_$(date +%Y%m%d_%H%M%S).tgz
tar czf "$OUT" $LOGS $RUN
echo "Packed: $LOGS $RUN"
echo "scp ubuntu@$(hostname -I | awk '{print $1}'):~/hw3_runs/$OUT ~/dev/CSE276/CSE276Robot/runs/"
