#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

export PYTHONPATH="/app/robotic-arm-mcp:${PYTHONPATH:-}"

cleanup() {
  if [[ -n "${SIM_PID:-}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    kill "${SIM_PID}" 2>/dev/null || true
    wait "${SIM_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

cd /app/robotic-arm-mcp
python sim/sim_driver_node.py &
SIM_PID=$!

sleep 2

python server.py --backend "${MCP_BACKEND:-ros2}" --host "${MCP_HOST:-0.0.0.0}" --port "${MCP_PORT:-8010}"
