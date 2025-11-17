#!/usr/bin/env bash
#
# Robot-side bringup (frontend) for distributed Nav2 + explorer setup using Zenoh (rmw_zenoh_cpp).
# - Starts the Zenoh router (rmw_zenohd) on the robot (unless one is already running).
# - Starts your robot bringup (sensors, base controller, TF, etc.) using rmw_zenoh_cpp.
#
# Prereqs on this machine:
#   sudo apt install ros-humble-rmw-zenoh-cpp
#
# Edit the CONFIG section to match your system.

set -Ee -o pipefail

########## CONFIG ##########

# Shared ROS 2 domain ID (must match backend)
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-42}"

# Path to your ROS 2 workspace (containing your robot bringup & any extra packages)
ROS_WS="${ROS_WS:-./}"

# Command that starts your robot's low-level stack.
# Change this to your real bringup (e.g. turtlebot3_bringup, custom launch, etc.).
ROBOT_BRINGUP_CMD="${ROBOT_BRINGUP_CMD:-ros2 launch turtlebot3_bringup robot.launch.py}"

# Whether to start a local Zenoh router on this machine. If another router is
# already running on TCP 7447, this script will detect it and skip regardless.
START_ZENOH_ROUTER="${START_ZENOH_ROUTER:-true}"

# Optional extra args for rmw_zenohd (e.g., custom config or port).
# Example to use a different port: "--listen tcp/[::]:7448"
ZENOH_ROUTER_ARGS="${ZENOH_ROUTER_ARGS:-}"

# If you're using TurtleBot3 bringup, many launch files expect TB3_MODEL and sometimes LDS_MODEL.
# Set sensible defaults here, or leave empty to keep your existing environment values.
# Valid TB3_MODEL: burger | waffle | waffle_pi
TB3_MODEL_VALUE="${TB3_MODEL_VALUE:-burger}"
# Valid LDS_MODEL (varies by distro/driver): LDS-01 | LDS-02 (use what your robot has)
LDS_MODEL_VALUE="${LDS_MODEL_VALUE:-LDS-01}"

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
  # Temporarily relax nounset while sourcing ament setup files that expect unset vars
  set +u 2>/dev/null || true
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
# Restore nounset for the remainder of the script
set -u

PIDS=()

start_bg() {
  echo "[robot] Starting: $*"
  bash -lc "$*" &
  PIDS+=($!)
}

port_in_use() {
  local port="$1"
  # ss is widely available on modern Linux; fallback to netstat if needed
  if command -v ss >/dev/null 2>&1; then
    ss -ltn | grep -q ":${port} "
  else
    netstat -ltn 2>/dev/null | grep -q ":${port} " || return 1
  fi
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

# 1. Start the Zenoh router (rmw_zenohd) unless one is already running.
#    Default router config listens on tcp/[::]:7447 and handles discovery for rmw_zenoh.
ROUTER_PORT=7447
if [[ "${START_ZENOH_ROUTER}" == "true" ]]; then
  if pgrep -f ".*rmw_zenohd" >/dev/null 2>&1 || port_in_use "$ROUTER_PORT"; then
    echo "[robot][INFO] Zenoh router appears to be running already (pid or port ${ROUTER_PORT} in use). Skipping start."
  else
    start_bg "ros2 run rmw_zenoh_cpp rmw_zenohd ${ZENOH_ROUTER_ARGS}"
    # Small delay to let the router come up
    sleep 2
  fi
else
  echo "[robot][INFO] START_ZENOH_ROUTER=false — not starting a local router."
fi

# 1.5 Export TurtleBot3 env if the bringup is TurtleBot3 and values are set here
if [[ "$ROBOT_BRINGUP_CMD" == *"turtlebot3_bringup"* ]]; then
  if [[ -n "${TB3_MODEL_VALUE}" ]]; then
    export TB3_MODEL="${TB3_MODEL_VALUE}"
    echo "[robot] TB3_MODEL: ${TB3_MODEL}"
  fi
  if [[ -n "${LDS_MODEL_VALUE}" ]]; then
    export LDS_MODEL="${LDS_MODEL_VALUE}"
    echo "[robot] LDS_MODEL: ${LDS_MODEL}"
  fi
fi

# 2. Start your robot bringup (talking to the local Zenoh router).
start_bg "$ROBOT_BRINGUP_CMD"

echo "[robot] Zenoh router + robot bringup started."
echo "[robot] PIDs: ${PIDS[*]}"
echo "[robot] Press Ctrl+C to stop everything."

wait
echo "[robot] All robot-side processes exited."
