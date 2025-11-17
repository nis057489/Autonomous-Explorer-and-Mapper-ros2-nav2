#!/usr/bin/env bash
#
# Backend bringup for distributed Nav2 + SLAM Toolbox + custom_explorer using Zenoh (rmw_zenoh_cpp).
# - Connects backend ROS 2 nodes as Zenoh clients to a Zenoh router (on robot or local).
# - Starts SLAM Toolbox, Nav2, custom_explorer, and optionally Gazebo and RViz.
#
# Prereqs on this machine:
#   sudo apt install ros-humble-rmw-zenoh-cpp
#   # For simulation (Gazebo + TurtleBot3):
#   #   sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
#
# Edit the CONFIG section to match your system.

set -Ee -o pipefail

########## CONFIG ##########

# Shared ROS 2 domain ID (must match robot/viewer)
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-42}"

# Path to your ROS 2 workspace (containing Autonomous-Explorer-and-Mapper-ros2-nav2 etc.)
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"

# When using a router on the ROBOT, set its reachable IP here and keep START_LOCAL_ROUTER=false.
# When using a router on this BACKEND, set START_LOCAL_ROUTER=true and adjust CONNECT_ENDPOINTS accordingly.
ROBOT_WG_IP="${ROBOT_WG_IP:-192.168.100.21}"

# Port where rmw_zenohd listens (default 7447 in the default router config)
ROBOT_ZENOH_PORT="${ROBOT_ZENOH_PORT:-7447}"

# Nav2 params file (set to your tuned nav2_params.yaml, or leave empty to use Nav2 defaults)
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-}"   # e.g. "$HOME/ros2_ws/src/my_robot_nav2/config/nav2_params.yaml"

# Whether to run RViz automatically (true/false)
RUN_RVIZ="${RUN_RVIZ:-false}"

# Simulation mode: if true, start TurtleBot3 Gazebo and set use_sim_time
SIM_MODE="${SIM_MODE:-true}"

# Gazebo world/launch for TurtleBot3; empty uses TB3 default empty world
GAZEBO_LAUNCH="${GAZEBO_LAUNCH:-ros2 launch turtlebot3_gazebo empty_world.launch.py}"

# TB3 model for sim; Gazebo usually uses TURTLEBOT3_MODEL, but we set both for compatibility
TB3_MODEL_VALUE="${TB3_MODEL_VALUE:-waffle_pi}"
TURTLEBOT3_MODEL_VALUE="${TURTLEBOT3_MODEL_VALUE:-waffle_pi}"

# Router on backend? If true, start local rmw_zenohd here and listen on BACKEND_ZENOH_PORT
START_LOCAL_ROUTER="${START_LOCAL_ROUTER:-false}"
BACKEND_ZENOH_PORT="${BACKEND_ZENOH_PORT:-7447}"

# If set, will override Zenoh endpoints for clients. Provide a comma-separated list of endpoints,
# e.g., "tcp/ROBOT_IP:7447" or "tcp/BACKEND_IP:7447"
CONNECT_ENDPOINTS="${CONNECT_ENDPOINTS:-}"

######## END CONFIG ########

ROBOT_ENDPOINT="tcp/${ROBOT_WG_IP}:${ROBOT_ZENOH_PORT}"

echo "[backend] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[backend] ROS workspace: $ROS_WS"
[ -n "$CONNECT_ENDPOINTS" ] && echo "[backend] Zenoh client endpoints: $CONNECT_ENDPOINTS"
[ -z "$CONNECT_ENDPOINTS" ] && echo "[backend] Robot Zenoh endpoint: $ROBOT_ENDPOINT"
[ -n "$NAV2_PARAMS_FILE" ] && echo "[backend] Using Nav2 params file: $NAV2_PARAMS_FILE"
echo "[backend] Using rmw_zenoh_cpp as middleware."
echo "[backend] SIM_MODE: $SIM_MODE"
echo "[backend] RUN_RVIZ: $RUN_RVIZ"
echo "[backend] START_LOCAL_ROUTER: $START_LOCAL_ROUTER"

export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_LOCALHOST_ONLY=0

# Configure all ROS 2 nodes started from this script to act as Zenoh CLIENTS and connect to a router.
if [ -n "$CONNECT_ENDPOINTS" ]; then
  # convert comma-separated list to quoted array: a,b -> ["a","b"]
  CEP=$(echo "$CONNECT_ENDPOINTS" | sed 's/,/","/g')
  export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${CEP}\"]"
else
  export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ROBOT_ENDPOINT}\"]"
fi

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

# 0. Optionally start a local Zenoh router so the viewer can connect here directly
if [[ "$START_LOCAL_ROUTER" == "true" ]]; then
  if pgrep -f ".*rmw_zenohd" >/dev/null 2>&1 || ss -ltn | grep -q ":${BACKEND_ZENOH_PORT} "; then
    echo "[backend][INFO] Zenoh router appears to be running already (pid or port ${BACKEND_ZENOH_PORT}). Skipping start."
  else
    echo "[backend] Starting local Zenoh router on port ${BACKEND_ZENOH_PORT}"
    start_bg "ros2 run rmw_zenoh_cpp rmw_zenohd --listen tcp/[::]:${BACKEND_ZENOH_PORT}"
    sleep 2
  fi
fi

# 1. (SIM) Start Gazebo TurtleBot3, if SIM_MODE=true
if [[ "$SIM_MODE" == "true" ]]; then
  export TURTLEBOT3_MODEL="$TURTLEBOT3_MODEL_VALUE"
  export TB3_MODEL="$TB3_MODEL_VALUE"
  echo "[backend] TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
  echo "[backend] TB3_MODEL: $TB3_MODEL"
  start_bg "$GAZEBO_LAUNCH"
  # Give Gazebo a moment to start publishing /clock
  sleep 3
fi

# 2. SLAM Toolbox (online asynchronous mapping)
SLAM_CMD="ros2 launch slam_toolbox online_async_launch.py"
if [[ "$SIM_MODE" == "true" ]]; then
  SLAM_CMD+=" use_sim_time:=true"
fi
start_bg "$SLAM_CMD"

# 3. Nav2 bringup (using params file if provided)
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

# 4. Custom explorer node from Autonomous-Explorer-and-Mapper-ros2-nav2
#    (assumes the package 'custom_explorer' is built in $ROS_WS)
start_bg "ros2 run custom_explorer explorer"

# 5. RViz for visualization (optional on backend; viewer typically runs RViz)
if [ "$RUN_RVIZ" = "true" ]; then
  start_bg "ros2 launch nav2_bringup rviz_launch.py"
fi

echo "[backend] All backend nodes started."
echo "[backend] PIDs: ${PIDS[*]}"
echo "[backend] Press Ctrl+C to stop everything."

wait
echo "[backend] All backend processes exited."
