#!/usr/bin/env bash
#
# Robot-side bringup (frontend) for distributed Nav2 + explorer setup using Zenoh (rmw_zenoh_cpp).
# - Starts the Zenoh router (rmw_zenohd) on the robot.
# - Starts your robot bringup (sensors, base controller, TF, etc.) using rmw_zenoh_cpp.
#
# Prereqs on this machine:
#   sudo apt install ros-humble-rmw-zenoh-cpp
#
# Edit the CONFIG section to match your system.

set -e

########## CONFIG ##########

# Shared ROS 2 domain ID (must match backend)
ROS_DOMAIN_ID_VALUE="42"

# Path to your ROS 2 workspace (containing your robot bringup & any extra packages)
ROS_WS="$HOME/ros2_ws"

# Command that starts your robot's low-level stack.
# Change this to your real bringup (e.g. turtlebot3_bringup, custom launch, etc.).
ROBOT_BRINGUP_CMD="ros2 launch turtlebot3_bringup robot.launch.py"

######## END CONFIG ########

echo "[robot] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[robot] ROS workspace: $ROS_WS"
echo "[robot] Robot bringup command: $ROBOT_BRINGUP_CMD"
echo "[robot] Using rmw_zenoh_cpp as middleware."

export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_LOCALHOST_ONLY=0   # harmless with rmw_zenoh but keeps env consistent

if [ ! -d "$ROS_WS" ]; then
  echo "[robot][WARN] ROS workspace '$ROS_WS' not found. Edit ROS_WS in this script if needed."
fi

# Source ROS and workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
else
  echo "[robot][ERROR] /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  exit 1
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$ROS_WS/install/setup.bash"
else
  echo "[robot][WARN] $ROS_WS/install/setup.bash not found. Did you run 'colcon build'?"
fi

PIDS=()

start_bg() {
  echo "[robot] Starting: $*"
  bash -lc "$*" &
  PIDS+=($!)
}

cleanup() {
  echo
  echo "[robot] Caught signal, stopping robot processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" > /dev/null 2>&1; then
      kill "$pid" || true
    fi
  done
  wait || true
  echo "[robot] Shutdown complete."
}
trap cleanup INT TERM

########## START PROCESSES ##########

# 1. Start the Zenoh router (rmw_zenohd).
#    Default router config listens on tcp/[::]:7447 and handles discovery for rmw_zenoh. :contentReference[oaicite:3]{index=3}
start_bg "ros2 run rmw_zenoh_cpp rmw_zenohd"

# Small delay to let the router come up
sleep 2

# 2. Start your robot bringup (talking to the local Zenoh router).
start_bg "$ROBOT_BRINGUP_CMD"

echo "[robot] Zenoh router + robot bringup started."
echo "[robot] PIDs: ${PIDS[*]}"
echo "[robot] Press Ctrl+C to stop everything."

wait
echo "[robot] All robot-side processes exited."
