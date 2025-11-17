#!/usr/bin/env bash
#
# Frontend bringup for distributed ROS2 system using Zenoh (rmw_zenoh_cpp).
# - Runs visualization (RVIZ) and simulation (Gazebo with TurtleBot3).
# - Connects to backend Zenoh router for heavy compute (SLAM, Nav2, Explorer).
#
# Prereqs on this machine:
#   sudo apt install ros-humble-rmw-zenoh-cpp
#   sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3*
#   sudo apt install ros-humble-nav2-bringup
#
# Edit the CONFIG section to match your system.

set -Ee -o pipefail

########## CONFIG ##########

# Shared ROS 2 domain ID (must match backend)
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-42}"

# Backend IP address (where backend.sh is running)
BACKEND_IP="${BACKEND_IP:-192.168.1.100}"

# Port where backend Zenoh router listens (default 7447)
BACKEND_ZENOH_PORT="${BACKEND_ZENOH_PORT:-7447}"

# Whether to run Gazebo simulation (true/false)
RUN_GAZEBO="${RUN_GAZEBO:-true}"

# Gazebo world/launch for TurtleBot3
GAZEBO_LAUNCH="${GAZEBO_LAUNCH:-ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py}"

# Whether to run RViz for visualization (true/false)
RUN_RVIZ="${RUN_RVIZ:-true}"

# TB3 model for simulation
TB3_MODEL_VALUE="${TB3_MODEL_VALUE:-waffle_pi}"
TURTLEBOT3_MODEL_VALUE="${TURTLEBOT3_MODEL_VALUE:-waffle_pi}"

######## END CONFIG ########

BACKEND_ENDPOINT="tcp/${BACKEND_IP}:${BACKEND_ZENOH_PORT}"

echo "[frontend] ===== FRONTEND (Visualization & Simulation) ====="
echo "[frontend] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[frontend] Backend Zenoh endpoint: $BACKEND_ENDPOINT"
echo "[frontend] Using rmw_zenoh_cpp as middleware (Client mode)"
echo "[frontend] RUN_GAZEBO: $RUN_GAZEBO"
echo "[frontend] RUN_RVIZ: $RUN_RVIZ"

export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_LOCALHOST_ONLY=0

# Frontend nodes act as Zenoh CLIENTS connecting to backend router
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${BACKEND_ENDPOINT}\"]"

# Source ROS
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u 2>/dev/null || true
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[frontend][ERROR] /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  exit 1
fi

PIDS=()

start_bg() {
  echo "[frontend] Starting: $*"
  bash -lc "$*" &
  PIDS+=($!)
}

cleanup() {
  echo
  echo "[frontend] Caught signal, stopping frontend nodes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" > /dev/null 2>&1; then
      kill "$pid" || true
    fi
  done
  wait || true
  echo "[frontend] Shutdown complete."
}
trap cleanup INT TERM

########## START NODES ##########

echo "[frontend] Connecting to backend at $BACKEND_ENDPOINT..."

# 1. Start Gazebo TurtleBot3 simulation (if enabled)
if [[ "$RUN_GAZEBO" == "true" ]]; then
  export TURTLEBOT3_MODEL="$TURTLEBOT3_MODEL_VALUE"
  export TB3_MODEL="$TB3_MODEL_VALUE"
  echo "[frontend] TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
  echo "[frontend] Starting Gazebo simulation..."
  start_bg "$GAZEBO_LAUNCH"
  # Give Gazebo a moment to start publishing /clock
  sleep 5
fi

# 2. Start RViz for visualization (if enabled)
if [[ "$RUN_RVIZ" == "true" ]]; then
  echo "[frontend] Starting RViz..."
  start_bg "ros2 launch nav2_bringup rviz_launch.py"
  sleep 2
fi

echo "[frontend] All frontend nodes started (Gazebo, RViz)."
echo "[frontend] PIDs: ${PIDS[*]}"
echo "[frontend] Connected to backend at: $BACKEND_ENDPOINT"
echo "[frontend] Press Ctrl+C to stop everything."

wait
echo "[frontend] All frontend processes exited."
