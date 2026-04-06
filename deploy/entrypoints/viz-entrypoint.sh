#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

export PYTHONPATH="/app/robotic-arm-mcp:${PYTHONPATH:-}"
export VIZ_MODE="${VIZ_MODE:-x11}"
export NOVNC_PORT="${NOVNC_PORT:-6080}"

cleanup() {
  for pid_var in RVIZ_PID WS_PID VNC_PID XVFB_PID; do
    pid="${!pid_var:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

cd /app/robotic-arm-mcp

if [[ "${VIZ_MODE}" == "novnc" ]]; then
  export DISPLAY="${DISPLAY:-:1}"
  Xvfb "${DISPLAY}" -screen 0 1280x800x24 &
  XVFB_PID=$!
  sleep 1

  python3 sim/sim_viewer.py &
  RVIZ_PID=$!

  x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport 5900 -listen 0.0.0.0 &
  VNC_PID=$!

  websockify --web=/usr/share/novnc/ "${NOVNC_PORT}" localhost:5900 &
  WS_PID=$!

  wait -n "${RVIZ_PID}" "${VNC_PID}" "${WS_PID}"
else
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY no esta configurado. Usa VIZ_MODE=novnc o exporta DISPLAY." >&2
    exit 1
  fi

  python3 sim/sim_viewer.py
fi
