#!/bin/bash -l
# Start / stop a headless Newton sim for RTF measurements (B8).
#
#   rtf_sim.sh stop                 stop EVERY running app.launch + engine
#   rtf_sim.sh start <tag>          start headless, wait until the engine runs
#   rtf_sim.sh engines              list running engines (must be <= 1)
#
# Env: LAUNCHER (default launcher_newton_mjwarp), EXTRA (extra launch args),
#      OUT (log dir, default /tmp/rtfbench).  Log: $OUT/launch_<tag>.log
OUT=${OUT:-/tmp/rtfbench}

stop() {
  pkill -INT -f "ros2 launch genie_sim_bringup app.launch.py" 2>/dev/null
  for i in $(seq 1 30); do
    pgrep -f "genie_sim_engine_newton.py" >/dev/null || break
    sleep 1
  done
  pkill -KILL -f "genie_sim_engine_newton.py" 2>/dev/null
  pkill -KILL -f "ros2 launch genie_sim_bringup app.launch.py" 2>/dev/null
  sleep 1
  pgrep -af "genie_sim_engine_newton.py|app.launch.py" || echo "stopped"
}

start() {
  mkdir -p "$OUT"
  source /workspace/devel/setup.bash
  local log="$OUT/launch_$1.log"
  # app.launch.py asks for ENTER on newton launchers and aborts on EOF:
  # send one newline, then keep stdin open.
  (echo; sleep infinity) | setsid ros2 launch genie_sim_bringup app.launch.py scene:=scene_pnp_g2_op \
      launcher_config:=${LAUNCHER:-launcher_newton_mjwarp} headless:=true ${EXTRA:-} >"$log" 2>&1 &
  for i in $(seq 1 600); do
    grep -q "physics=" "$log" 2>/dev/null && { echo "up after ${i}s, log=$log"; return 0; }
    sleep 1
  done
  echo "TIMEOUT"; tail -30 "$log"; return 1
}

engines() {
  pgrep -af "genie_sim_engine_newton.py" | cut -c1-160 || echo "no engine running"
}

"$@"
