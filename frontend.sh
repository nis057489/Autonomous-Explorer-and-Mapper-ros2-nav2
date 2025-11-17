#!/usr/bin/env bash
#
# Robot-side bringup (frontend) for distributed Nav2 + explorer setup over WireGuard.
# - Binds CycloneDDS to the WireGuard interface (wg0 by default)
# - Starts your robot bringup (sensors, base controller, TF, etc.).
#
# Edit the CONFIG section to match your system.

set -e

########## CONFIG ##########

# WireGuard network interface name
WG_IF="wg0"

# Shared ROS 2 domain ID (must match backend)
ROS_DOMAIN_ID_VALUE="42"

# Path to your ROS 2 workspace (containing Autonomous-Explorer-and-Mapper-ros2-nav2)
ROS_WS="$HOME/ros2_ws"

# CycloneDDS config file location (per-user, no sudo needed)
CYCLONEDDS_CONFIG_FILE="$HOME/.config/cyclonedds_robot.xml"

# Command that starts your robot's low-level stack.
# Change this to your real bringup (e.g. turtlebot3_bringup, custom launch, etc.).
ROBOT_BRINGUP_CMD="ros2 launch turtlebot3_bringup robot.launch.py"

######## END CONFIG ########

echo "[robot] Using WireGuard interface: $WG_IF"
echo "[robot] ROS_DOMAIN_ID: $ROS_DOMAIN_ID_VALUE"
echo "[robot] ROS workspace: $ROS_WS"
echo "[robot] CycloneDDS config: $CYCLONEDDS_CONFIG_FILE"
echo "[robot] Robot bringup command: $ROBOT_BRINGUP_CMD"

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
  echo "[robot][WARN] Interface $WG_IF not found. Is WireGuard up?"
fi

if [ ! -d "$ROS_WS" ]; then
  echo "[robot][ERROR] ROS workspace '$ROS_WS' not found. Edit ROS_WS in this script."
  exit 1
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

echo "[robot] Environment configured. Starting robot bringup..."

# Trap Ctrl+C so we can cleanly stop the bringup
cleanup() {
  echo
  echo "[robot] Caught signal, stopping robot bringup..."
  # Kill all background jobs started by this script
  jobs -p | xargs -r kill
  wait || true
  echo "[robot] Shutdown complete."
}
trap cleanup INT TERM

# Start your robot bringup (blocking)
eval "$ROBOT_BRINGUP_CMD"

# If bringup exits, just finish script
echo "[robot] Robot bringup exited."
