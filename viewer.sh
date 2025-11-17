#!/usr/bin/env bash
#
# Operator-side visualization (RViz) for distributed Nav2 + explorer using Zenoh (rmw_zenoh_cpp).
# - Does NOT start robot hardware drivers.
# - Optionally starts a local Zenoh router (disabled by default). Typically you connect to the robot's router.
#
# Prereqs on this machine:
#   sudo apt install ros-humble-rmw-zenoh-cpp rviz2
#   # Optional for Gazebo sim viewing:
#   # sudo apt install ros-humble-turtlebot3-gazebo
#
# You can override any CONFIG var with environment variables when invoking the script.

set -Ee -o pipefail

########## CONFIG ##########

# Shared ROS 2 domain ID (must match robot/backend)
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-42}"

# Path to local ROS 2 workspace (if you have overlays); leave as ./ if none
ROS_WS="${ROS_WS:-./}"

# RViz config file (optional). If empty or missing, RViz starts with default layout.
RVIZ_CONFIG="${RVIZ_CONFIG:-}"

# Whether to start a local Zenoh router here. Default: false (use the robot's router)
START_ZENOH_ROUTER="${START_ZENOH_ROUTER:-false}"

# Extra args for rmw_zenohd (e.g., "--listen tcp/[::]:7448").
ZENOH_ROUTER_ARGS="${ZENOH_ROUTER_ARGS:-}"

# If set, the viewer will explicitly connect as a Zenoh CLIENT to these endpoints (comma-separated)
# Example: ZENOH_CONNECT="tcp/192.168.1.10:7447"
ZENOH_CONNECT="${ZENOH_CONNECT:-}"

# Extra args for RViz (e.g., "-d /path/to/config.rviz")
RVIZ_ARGS="${RVIZ_ARGS:-}"

######## END CONFIG ########

echo "[viewer] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[viewer] ROS workspace: $ROS_WS"

export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_LOCALHOST_ONLY=0

# If provided by your network setup, you may specify known router locators, e.g.:
# export ZENOH_ROUTER_LOCATORS="tcp/robot-hostname-or-ip:7447"
# Or just set ZENOH_CONNECT above and we will build the override:
if [[ -n "$ZENOH_CONNECT" ]]; then
  CEP=$(echo "$ZENOH_CONNECT" | sed 's/,/","/g')
  export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${CEP}\"]"
  echo "[viewer] Using Zenoh endpoints: $ZENOH_CONNECT"
fi

# Source ROS and workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u 2>/dev/null || true
  source /opt/ros/humble/setup.bash
else
  echo "[viewer][ERROR] /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  exit 1
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$ROS_WS/install/setup.bash"
else
  echo "[viewer][INFO] $ROS_WS/install/setup.bash not found. Proceeding without overlay."
fi
set -u

PIDS=()

start_bg() {
  echo "[viewer] Starting: $*"
  bash -lc "$*" &
  PIDS+=($!)
}

cleanup() {
  echo
  echo "[viewer] Caught signal, stopping processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" || true
    fi
  done
  wait || true
  echo "[viewer] Shutdown complete."
}
trap cleanup INT TERM

port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn | grep -q ":${port} "
  else
    netstat -ltn 2>/dev/null | grep -q ":${port} " || return 1
  fi
}

# Optionally start Zenoh router
ROUTER_PORT=7447
if [[ "${START_ZENOH_ROUTER}" == "true" ]]; then
  if pgrep -f ".*rmw_zenohd" >/dev/null 2>&1 || port_in_use "$ROUTER_PORT"; then
    echo "[viewer][INFO] Zenoh router appears running already (pid or port ${ROUTER_PORT}). Skipping."
  else
    start_bg "ros2 run rmw_zenoh_cpp rmw_zenohd ${ZENOH_ROUTER_ARGS}"
    sleep 2
  fi
else
  echo "[viewer][INFO] Not starting a local Zenoh router (START_ZENOH_ROUTER=false)."
fi

# Build RViz args
if [[ -n "$RVIZ_CONFIG" && -f "$RVIZ_CONFIG" ]]; then
  RVIZ_ARGS="${RVIZ_ARGS} -d \"$RVIZ_CONFIG\""
  echo "[viewer] Using RViz config: $RVIZ_CONFIG"
fi

# Start RViz2
start_bg "rviz2 ${RVIZ_ARGS}"

echo "[viewer] RViz started. PIDs: ${PIDS[*]}"
echo "[viewer] Press Ctrl+C to stop."

wait

echo "[viewer] All processes exited."
