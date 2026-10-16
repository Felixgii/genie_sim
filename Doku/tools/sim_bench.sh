#!/bin/bash -l
# Startet eine Szene, wartet auf laufende Physik, misst RTF / GPU / Engine-Stats, stoppt alles.
#
#   bash /workspace/Doku/tools/sim_bench.sh <tag> <warmup_s> <ros2-launch-args...>
#   z.B. sim_bench.sh newton_flat 60 scene:=scene_flat_g2_sp launcher_config:=launcher_newton_mjwarp headless:=true
#
# Im Container als isaac-sim (Login-Shell, sonst fehlt die DDS-Umgebung):
#   docker exec -u isaac-sim geniesim3 bash -lc 'bash /workspace/Doku/tools/sim_bench.sh ...'
# GPU-Zeit pro Physik-Schritt: vorher `export GENIESIM_STEP_GPU_SYNC=1` (nur Newton).
# Log: /tmp/gs/bench_<tag>.log, GPU-Samples: /tmp/gs/gpu_<tag>.csv
tag=$1; warm=$2; shift 2
here=$(dirname "$(readlink -f "$0")")
out=/tmp/gs; mkdir -p $out
log=$out/bench_$tag.log
source /workspace/devel/setup.bash

if ps -eo cmd | grep -E '^[^ ]*python3? .*genie_sim_engine_(isaacsim|newton)\.py' | grep -qv grep; then
  echo "ENGINE LAEUFT BEREITS - erst stoppen"; exit 1
fi

# Newton-Launcher fragen per ENTER nach; ein Newline schicken, stdin offen halten.
(echo; sleep infinity) | setsid ros2 launch genie_sim_bringup app.launch.py "$@" > $log 2>&1 &
for i in $(seq 1 900); do
  grep -qE "\[stage\] active|Physics Stats" $log && break
  grep -q "genie_sim_engine_[a-z]*\.py-[0-9]*\]: process has died" $log && { echo "ENGINE DIED"; tail -20 $log; exit 1; }
  sleep 1
done
echo "up after ${i}s"; sleep $warm

nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits -l 1 > $out/gpu_$tag.csv &
gpu_pid=$!
python3 $here/rtf_clock.py 25
kill $gpu_pid
awk -F", " '{u+=$1; m+=$2; n++} END {printf "== GPU util mean %.0f %% (n=%d), mem mean %.0f MiB\n", u/n, n, m/n}' $out/gpu_$tag.csv
echo "== image topics: $(ros2 topic list | grep -c image_raw)"
echo "== engine stats (last 2 blocks)"
sed "s/\x1b\[[0-9;]*m//g" $log | grep -E "Actual Hz|Interval: mean|Solver: |Publish: |step GPU sync" | grep -v render_node | tail -8

echo "== stopping"
pkill -INT -f "ros2 launch genie_sim_bringup"
for i in $(seq 1 40); do pgrep -f "genie_sim_engine_|genie_sim_render" >/dev/null || break; sleep 1; done
# Was SIGINT ueberlebt (Render-Node, robot_state_publisher, Servo), hart beenden.
pkill -KILL -f "genie_sim_engine_|genie_sim_render|app.launch.py|robot_state_publisher|servo_node|sleep infinity"
sleep 1
pgrep -af "app.launch|robot_state|genie_sim_" | grep -v pgrep && echo "LEFTOVER" || echo "stopped"
