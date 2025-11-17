#!/usr/bin/env bash
#
# Backend bringup for distributed Nav2 + SLAM Toolbox + custom_explorer.
# - Binds CycloneDDS to the WireGuard interface (wg0 by default)
# - Starts SLAM Toolbox
# - Starts Nav2
# - Starts the custom explorer node (from Autonomous-Explorer-and-Mapper-ros2-nav2)
# - Optionally starts RViz for visualization.
#
# Edit the CONFIG section to match your system.

set -e

########## CONFIG ##########

# WireGuard network interface name
WG_IF="wg0"

# Shared ROS 2 domain ID (must match robot)
ROS_DOMAIN_ID_VALUE="42"

# Path to your ROS 2 workspace (containing Autonomous-Explorer-and-Mapper-ros2-nav2)
ROS_WS="$HOME/ros2_ws"

# CycloneDDS config file location (per-user, no sudo needed)
CYCLONEDDS_CONFIG_FILE="$HOME/.config/cyclonedds_backend.xml"

# Nav2 params file for your real robot.
# Set this to your tuned nav2_params.yaml, or leave empty to use Nav2 defaults.
NAV2_PARAMS_FILE=""   # e.g. "$HOME/ros2_ws/src/my_robot_nav2/config/nav2_params.yaml"

# Whether to run RViz automatically (true/false)
RUN_RVIZ="true"

######## END CONFIG ########

echo "[backend] Using WireGuard interface: $WG_IF"
echo "[backend] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[backend] ROS workspace: $ROS_WS"
echo "[backend] CycloneDDS config: $CYCLONEDDS_CONFIG_FILE"
[ -n "$NAV2_PARAMS_FILE" ] && echo "[backend] Using Nav2 params file: $NAV2_PARAMS_FILE"

# Ensure config dir exists
mkdir -p "$(dirname "$CYCLONEDDS_CONFIG_FILE")"

# Create CycloneDDS config bound to the WireGuard interface
cat > "$CYCLONEDDS_CONFIG_FILE" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain id="any">
    <General>
      <!-- Bind DDS to the WireGuard interface -->
      <NetworkInterfaceAddress>${WG_IF}</NetworkInterfaceAddress>
      <!-- Usually good over VPNs -->
      <AllowMulticast>false</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI="file://$CYCLONEDDS_CONFIG_FILE"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
export ROS_LOCALHOST_ONLY=0

# Basic checks
if ! ip link show "$WG_IF" > /dev/null 2>&1; then
  echo "[backend][WARN] Interface $WG_IF not found. Is WireGuard up?"
fi

if [ ! -d "$ROS_WS" ]; then
  echo "[backend][ERROR] ROS workspace '$ROS_WS' not found. Edit ROS_WS in this script."
  exit 1
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

# Helper to run long-lived nodes in background and remember PIDs
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

# 1. SLAM Toolbox (online asynchronous mapping) :contentReference[oaicite:1]{index=1}
start_bg "ros2 launch slam_toolbox online_async_launch.py"

# 2. Nav2 bringup (using params file if provided) :contentReference[oaicite:2]{index=2}
NAV2_CMD="ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false"
if [ -n "$NAV2_PARAMS_FILE" ]; then
  NAV2_CMD="$NAV2_CMD params_file:=$NAV2_PARAMS_FILE"
fi
start_bg "$NAV2_CMD"

# 3. Custom explorer node from Autonomous-Explorer-and-Mapper-ros2-nav2 :contentReference[oaicite:3]{index=3}
#    This expects that you've cloned and built the repo in $ROS_WS/src
start_bg "ros2 run custom_explorer explorer"

# 4. RViz for visualization (optional, can be disabled via RUN_RVIZ=false) :contentReference[oaicite:4]{index=4}
if [ "$RUN_RVIZ" = "true" ]; then
  start_bg "ros2 launch nav2_bringup rviz_launch.py"
fi

echo "[backend] All backend nodes started."
echo "[backend] PIDs: ${PIDS[*]}"
echo "[backend] Press Ctrl+C to stop everything."

# Wait for all children
wait
echo "[backend] All backend processes exited."
