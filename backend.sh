#!/usr/bin/env bash
#
# Backend bringup for distributed Nav2 + SLAM Toolbox + custom_explorer using Zenoh (rmw_zenoh_cpp).
# - Runs heavy compute nodes: SLAM Toolbox, Nav2, and custom_explorer.
# - Acts as Zenoh router for frontend to connect.
# - Frontend (RVIZ/Gazebo) should run on a separate machine using frontend.sh
#
# Prereqs on this machine:
#   sudo apt install ros-humble-rmw-zenoh-cpp
#   sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox
#
# Edit the CONFIG section to match your system.

set -Ee -o pipefail

########## CONFIG ##########

# Shared ROS 2 domain ID (must match frontend)
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-42}"

# Path to your ROS 2 workspace (containing Autonomous-Explorer-and-Mapper-ros2-nav2 etc.)
ROS_WS="${ROS_WS:-/var/home/nick/code/Autonomous-Explorer-and-Mapper-ros2-nav2}"

# Backend IP address (where this script runs - for frontend to connect to)
BACKEND_IP="${BACKEND_IP:-0.0.0.0}"

# Port where rmw_zenohd listens (default 7447)
BACKEND_ZENOH_PORT="${BACKEND_ZENOH_PORT:-7447}"

# Nav2 params file (set to your tuned nav2_params.yaml, or leave empty to use Nav2 defaults)
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-}"

# Simulation mode: if true, use use_sim_time:=true for all nodes
SIM_MODE="${SIM_MODE:-true}"

# TB3 model for compatibility
TB3_MODEL_VALUE="${TB3_MODEL_VALUE:-waffle_pi}"
TURTLEBOT3_MODEL_VALUE="${TURTLEBOT3_MODEL_VALUE:-waffle_pi}"


######## END CONFIG ########

echo "[backend] ===== BACKEND (Heavy Compute) ====="
echo "[backend] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[backend] ROS workspace: $ROS_WS"
echo "[backend] Backend Zenoh port: $BACKEND_ZENOH_PORT"
echo "[backend] Using rmw_zenoh_cpp as middleware (Router mode)"
echo "[backend] SIM_MODE: $SIM_MODE"

export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_LOCALHOST_ONLY=0

# Backend runs as Zenoh ROUTER so frontend can connect
export ZENOH_CONFIG_OVERRIDE="mode=\"router\";listen/endpoints=[\"tcp/${BACKEND_IP}:${BACKEND_ZENOH_PORT}\"]"

if [ ! -d "$ROS_WS" ]; then
  echo "[backend][WARN] ROS workspace '$ROS_WS' not found. Edit ROS_WS in this script if needed."
fi

# Source ROS and workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u 2>/dev/null || true
  source /opt/ros/humble/setup.bash
else
  echo "[backend][ERROR] /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  exit 1
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$ROS_WS/install/setup.bash"
else
  echo "[backend][WARN] $ROS_WS/install/setup.bash not found. Did you run 'colcon build'?"
fi
set -u

PIDS=()

start_bg() {
  echo "[backend] Starting: $*"
  bash -lc "$*" &
  PIDS+=($!)
}

cleanup() {
  echo
  echo "[backend] Caught signal, stopping backend nodes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" > /dev/null 2>&1; then
      kill "$pid" || true
    fi
  done
  wait || true
  echo "[backend] Shutdown complete."
}
trap cleanup INT TERM

########## START NODES ##########

echo "[backend] Starting Zenoh router..."

# 0. Start local Zenoh router (backend acts as router)
if pgrep -f ".*rmw_zenohd" >/dev/null 2>&1 || ss -ltn | grep -q ":${BACKEND_ZENOH_PORT} "; then
  echo "[backend][INFO] Zenoh router appears to be running already (pid or port ${BACKEND_ZENOH_PORT}). Skipping start."
else
  echo "[backend] Starting Zenoh router on port ${BACKEND_ZENOH_PORT}"
  start_bg "ros2 run rmw_zenoh_cpp rmw_zenohd"
  sleep 3
fi

# 1. SLAM Toolbox (online asynchronous mapping) - HEAVY COMPUTE
SLAM_CMD="ros2 launch slam_toolbox online_async_launch.py"
if [[ "$SIM_MODE" == "true" ]]; then
  SLAM_CMD+=" use_sim_time:=true"
fi
start_bg "$SLAM_CMD"
sleep 2

# 2. Nav2 bringup (path planning, navigation) - HEAVY COMPUTE
NAV2_CMD="ros2 launch nav2_bringup navigation_launch.py"
if [[ "$SIM_MODE" == "true" ]]; then
  NAV2_CMD+=" use_sim_time:=true"
else
  NAV2_CMD+=" use_sim_time:=false"
fi
if [ -n "$NAV2_PARAMS_FILE" ]; then
  NAV2_CMD+=" params_file:=$NAV2_PARAMS_FILE"
fi
start_bg "$NAV2_CMD"
sleep 2

# 3. Custom explorer node - HEAVY COMPUTE (frontier detection, goal selection)
start_bg "ros2 run custom_explorer explorer"

echo "[backend] All backend nodes started (Zenoh router, SLAM, Nav2, Explorer)."
echo "[backend] PIDs: ${PIDS[*]}"
echo "[backend] Frontend machines should connect to: tcp://<BACKEND_IP>:${BACKEND_ZENOH_PORT}"
echo "[backend] Press Ctrl+C to stop everything."

wait
echo "[backend] All backend processes exited."
