#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
COLCON_ARGS=" --event-handlers console_direct+ --symlink-install " 


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

# CUDA: driver/runtime có sẵn không đủ — CMake cần thấy nvcc. Một số môi trường
# (IDE, task, shell tối giản) không có /usr/local/cuda trong PATH.
if [ -z "${CMAKE_CUDA_COMPILER:-}" ]; then
    for _nvcc in /usr/local/cuda/bin/nvcc /usr/local/cuda-12.6/bin/nvcc /usr/local/cuda-12/bin/nvcc; do
        if [ -x "$_nvcc" ]; then
            export CMAKE_CUDA_COMPILER="$_nvcc"
            export PATH="$(dirname "$_nvcc"):${PATH}"
            break
        fi
    done
fi
if [ -n "${CMAKE_CUDA_COMPILER:-}" ] && [ -x "${CMAKE_CUDA_COMPILER}" ]; then
    echo "=== CUDA compiler: ${CMAKE_CUDA_COMPILER} ==="
else
    echo "WARNING: nvcc not found; set CMAKE_CUDA_COMPILER or add CUDA bin to PATH before build."
fi

# ---------------------------------------------------------------------------
# Init submodules
# ---------------------------------------------------------------------------
cd "$SCRIPT_DIR"

echo "=== Initializing submodules ==="
git submodule update --init gtsam gtsam_points glim_ros2 

# NOTE:
# `scancontext` (inside `glim_ext`) currently contains a nested gitlink
# `example/.../npy-matlab` without a corresponding `.gitmodules` entry at that
# commit, which makes `--recursive` fail. We therefore init `glim_ext` without
# recursion, and recurse only into known-good dependencies that are required.
git submodule update --init --recursive glim iridescence
git submodule update --init glim_ext

# Init selected nested submodules inside `glim_ext`.
git -C "$SCRIPT_DIR/glim_ext" submodule update --init --recursive \
    modules/odometry/orb_slam_frontend/thirdparty/ORB_SLAM3
git -C "$SCRIPT_DIR/glim_ext" submodule update --init \
    modules/mapping/scan_context_loop_detector/thirdparty/scancontext

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
colcon build $COLCON_ARGS --packages-select gtsam \
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
colcon build $COLCON_ARGS --packages-select iridescence \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


# ---------------------------------------------------------------------------
# Build gtsam_points
# ---------------------------------------------------------------------------
echo "=== Building gtsam_points ==="
colcon build $COLCON_ARGS --packages-select gtsam_points \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    ${CMAKE_CUDA_COMPILER:+-DCMAKE_CUDA_COMPILER="${CMAKE_CUDA_COMPILER}"}

# shellcheck source=/dev/null
source "$WS_ROOT/install/setup.bash"

# ---------------------------------------------------------------------------
# Build glim
# ---------------------------------------------------------------------------
echo "=== Building glim ==="
colcon build $COLCON_ARGS --packages-select glim \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WITH_OPENCV=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    ${CMAKE_CUDA_COMPILER:+-DCMAKE_CUDA_COMPILER="${CMAKE_CUDA_COMPILER}"}

# shellcheck source=/dev/null
source "$WS_ROOT/install/setup.bash"

# ---------------------------------------------------------------------------
# Build glim_ext
# ---------------------------------------------------------------------------
echo "=== Building glim_ext ==="
colcon build --packages-select glim_ext \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


# ---------------------------------------------------------------------------
# Build glim_ros2
# ---------------------------------------------------------------------------
echo "=== Building glim_ros ==="
colcon build $COLCON_ARGS --packages-select glim_ros \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo ""
echo "=== Build complete! ==="
echo "Source the workspace: source $WS_ROOT/install/setup.bash"
