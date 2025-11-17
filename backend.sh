#!/usr/bin/env bash
#
# Backend bringup for distributed Nav2 + SLAM Toolbox + custom_explorer using Zenoh (rmw_zenoh_cpp).
# - Connects backend ROS 2 nodes as Zenoh clients to the robot's Zenoh router.
# - Starts SLAM Toolbox, Nav2, custom_explorer, and optionally RViz.
#
# Prereqs on this machine:
#   sudo apt install ros-humble-rmw-zenoh-cpp
#
# Edit the CONFIG section to match your system.

set -e

########## CONFIG ##########

# Shared ROS 2 domain ID (must match robot)
ROS_DOMAIN_ID_VALUE="42"

# Path to your ROS 2 workspace (containing Autonomous-Explorer-and-Mapper-ros2-nav2 etc.)
ROS_WS="$HOME/ros2_ws"

# WireGuard / VPN IP of the robot machine, where the Zenoh router is running
ROBOT_WG_IP="192.168.100.21"

# Port where rmw_zenohd listens (default 7447 in the default router config) :contentReference[oaicite:6]{index=6}
ROBOT_ZENOH_PORT="7447"

# Nav2 params file for your real robot.
# Set this to your tuned nav2_params.yaml, or leave empty to use Nav2 defaults.
NAV2_PARAMS_FILE=""   # e.g. "$HOME/ros2_ws/src/my_robot_nav2/config/nav2_params.yaml"

# Whether to run RViz automatically (true/false)
RUN_RVIZ="true"

######## END CONFIG ########

ROBOT_ENDPOINT="tcp/${ROBOT_WG_IP}:${ROBOT_ZENOH_PORT}"

echo "[backend] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[backend] ROS workspace: $ROS_WS"
echo "[backend] Robot Zenoh endpoint: $ROBOT_ENDPOINT"
[ -n "$NAV2_PARAMS_FILE" ] && echo "[backend] Using Nav2 params file: $NAV2_PARAMS_FILE"
echo "[backend] Using rmw_zenoh_cpp as middleware."

export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_LOCALHOST_ONLY=0

# Configure all ROS 2 nodes started from this script to:
#   - act as Zenoh CLIENTS
#   - connect directly to the robot's Zenoh router over TCP
# Based on rmw_zenoh session config examples (mode=client + connect/endpoints). :contentReference[oaicite:7]{index=7}
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ROBOT_ENDPOINT}\"]"

if [ ! -d "$ROS_WS" ]; then
  echo "[backend][WARN] ROS workspace '$ROS_WS' not found. Edit ROS_WS in this script if needed."
fi

# Source ROS and workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck source=/dev/null
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

# 1. SLAM Toolbox (online asynchronous mapping)
start_bg "ros2 launch slam_toolbox online_async_launch.py"

# 2. Nav2 bringup (using params file if provided)
NAV2_CMD="ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false"
if [ -n \"$NAV2_PARAMS_FILE\" ]; then
  NAV2_CMD=\"\$NAV2_CMD params_file:=$NAV2_PARAMS_FILE\"
fi
start_bg "$NAV2_CMD"

# 3. Custom explorer node from Autonomous-Explorer-and-Mapper-ros2-nav2
#    (assumes the package 'custom_explorer' is built in $ROS_WS)
start_bg "ros2 run custom_explorer explorer"

# 4. RViz for visualization (optional)
if [ \"$RUN_RVIZ\" = \"true\" ]; then
  start_bg \"ros2 launch nav2_bringup rviz_launch.py\"
fi

echo "[backend] All backend nodes started."
echo "[backend] PIDs: ${PIDS[*]}"
echo "[backend] Press Ctrl+C to stop everything."

wait
echo "[backend] All backend processes exited."
