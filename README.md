# 3D GLIM - Hướng dẫn cài đặt

Framework 3D Localization and Mapping dựa trên GLIM, đóng gói dưới dạng git submodules trong một workspace ROS 2.

## Yêu cầu hệ thống

### Build Tools
- **colcon**: Công cụ build chính cho ROS 2 workspace
- **CMake**: Hệ thống build cross-platform
- **C++ Compiler**: GCC hoặc Clang (C++14 trở lên)

### ROS 2
- ROS 2 Humble hoặc mới hơn

---

## Cài đặt nhanh

Toàn bộ quá trình clone submodule và build được tự động hóa qua script `build.sh`:

```bash
# Clone repo (nếu chưa có)
git clone <repo-url> ~/glim_3d_ws/src/3d_glim
cd ~/glim_3d_ws/src/3d_glim

# Chạy build script
chmod +x build.sh
./build.sh
```

Script sẽ tự động:
1. Detect và source môi trường ROS 2
2. Clone tất cả submodule cần thiết
3. Checkout GTSAM đúng phiên bản `4.3a0`
4. Build tuần tự: `gtsam` → `gtsam_points` → `glim` → `glim_ros`

---

## Cấu trúc dự án

```
3d_glim/
├── gtsam/          # Georgia Tech Smoothing and Mapping (tag 4.3a0)
├── gtsam_points/   # GTSAM extension cho point cloud
├── glim/           # 3D Localization and Mapping Framework
├── glim_ros2/      # ROS 2 interface cho GLIM
├── iridescence/    # Thư viện visualization (tùy chọn)
├── build.sh        # Script build tự động
└── README.md
```

---

## Cài đặt thủ công (từng bước)

### Bước 1 — Clone submodules

```bash
cd ~/glim_3d_ws/src/3d_glim

git submodule update --init gtsam gtsam_points glim_ros2
git submodule update --init --recursive glim iridescence

# Đảm bảo GTSAM đúng version
cd gtsam && git checkout 4.3a0 && cd ..
```

### Bước 2 — Build GTSAM

```bash
cd ~/glim_3d_ws
colcon build --packages-select gtsam \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF

source install/setup.bash
```

### Bước 3 — Build gtsam_points

```bash
colcon build --packages-select gtsam_points \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
```

### Bước 4 — Build GLIM

```bash
colcon build --packages-select glim \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/setup.bash
```

### Bước 5 — Build glim_ros2

```bash
colcon build --packages-select glim_ros \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Bước 6 — Build iridescence (tùy chọn, chỉ cần cho demo programs)

```bash
# Cài đặt system dependencies
sudo apt install -y libglm-dev libglfw3-dev libpng-dev

colcon build --packages-select iridescence \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

---

## Kiểm tra sau khi build

```bash
# Kiểm tra workspace đã được build
ls ~/glim_3d_ws/install/

# Source workspace
source ~/glim_3d_ws/install/setup.bash

# Kiểm tra các package ROS 2
ros2 pkg list | grep -E "gtsam|glim"
```

---

## Tùy chọn CMake nâng cao

### gtsam_points

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `BUILD_WITH_OPENMP` | ON | Enable OpenMP |
| `BUILD_WITH_TBB` | OFF | Enable TBB |
| `BUILD_WITH_CUDA` | OFF | Enable CUDA |
| `BUILD_WITH_MARCH_NATIVE` | OFF | Enable `-march=native` |
| `CMAKE_CUDA_ARCHITECTURES` | native | CUDA compute capability (vd: `89`) |
| `BUILD_DEMO` | OFF | Build demo programs |
| `BUILD_TESTS` | OFF | Build unit tests |
| `BUILD_TOOLS` | OFF | Build tools |
