#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=== Script dir : $SCRIPT_DIR ==="
echo "=== Workspace  : $WS_ROOT ==="

# Source ROS2 environment
ROS_SETUP=""
for distro in humble iron jazzy rolling; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        ROS_SETUP="/opt/ros/$distro/setup.bash"
        break
    fi
done

if [ -n "$ROS_SETUP" ]; then
    echo "=== Sourcing ROS2: $ROS_SETUP ==="
    # shellcheck source=/dev/null
    source "$ROS_SETUP"
else
    echo "WARNING: No ROS2 installation found in /opt/ros/"
fi

# ---------------------------------------------------------------------------
# Init submodules
# ---------------------------------------------------------------------------
cd "$SCRIPT_DIR"

echo "=== Initializing submodules ==="
git submodule update --init gtsam gtsam_points glim_ros2
git submodule update --init --recursive glim iridescence

# Checkout GTSAM 4.3a0
echo "=== Checkout GTSAM 4.3a0 ==="
cd "$SCRIPT_DIR/gtsam"
git checkout 4.3a0
cd "$SCRIPT_DIR"

# ---------------------------------------------------------------------------
# Build gtsam
# ---------------------------------------------------------------------------
echo "=== Building gtsam ==="
cd "$WS_ROOT"
colcon build --packages-select gtsam \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF

# shellcheck source=/dev/null
source "$WS_ROOT/install/setup.bash"

# ---------------------------------------------------------------------------
# Build iridescence
# ---------------------------------------------------------------------------
echo "=== Building iridescence ==="
sudo apt install -y libglm-dev libglfw3-dev libpng-dev
colcon build --packages-select iridescence \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


# ---------------------------------------------------------------------------
# Build gtsam_points
# ---------------------------------------------------------------------------
echo "=== Building gtsam_points ==="
colcon build --packages-select gtsam_points \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# shellcheck source=/dev/null
source "$WS_ROOT/install/setup.bash"

# ---------------------------------------------------------------------------
# Build glim
# ---------------------------------------------------------------------------
echo "=== Building glim ==="
colcon build --packages-select glim \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# shellcheck source=/dev/null
source "$WS_ROOT/install/setup.bash"

# ---------------------------------------------------------------------------
# Build glim_ros2
# ---------------------------------------------------------------------------
echo "=== Building glim_ros ==="
colcon build --packages-select glim_ros \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo ""
echo "=== Build complete! ==="
echo "Source the workspace: source $WS_ROOT/install/setup.bash"
