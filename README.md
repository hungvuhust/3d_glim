# 3D GLIM - Hướng dẫn cài đặt

Dự án này yêu cầu các công cụ build và thư viện sau để hoạt động:

## Yêu cầu hệ thống

### Build Tools
- **colcon**: Công cụ build chính cho ROS 2 workspace
- **CMake**: Hệ thống build cross-platform
- **C++ Compiler**: GCC hoặc Clang (C++14 trở lên)

### Thư viện chính

#### 1. GTSAM (Georgia Tech Smoothing and Mapping)
GTSAM là thư viện C++ cho việc tối ưu hóa và suy luận trong robotics và computer vision.

**Cài đặt GTSAM:**

```bash
# Clone GTSAM repository
git clone https://github.com/hungvuhust/gtsam.git
cd gtsam
git checkout 4.3a0 

# Về ros2_ws
cd ~/ros2_ws

# Cấu hình với CMake và build với colcon 
colcon build --packages-select gtsam \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_WITH_TBB=OFF \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
```

**Cài đặt thư viện visualization (tùy chọn):**
```bash
# Cài đặt dependencies cho iridescence
sudo apt install -y libglm-dev libglfw3-dev libpng-dev

# Clone và cài đặt iridescence (chỉ cần cho demo programs)
git clone https://github.com/hungvuhust/iridescence --recursive

# Về ros2_ws
cd ~/ros2_ws

# Cấu hình với CMake và build với colcon 
colcon build --packages-select iridescence \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
```

**Kiểm tra cài đặt:**
```bash
# Kiểm tra GTSAM đã được cài đặt
pkg-config --modversion gtsam
```

#### 2. gtsam_points
Thư viện mở rộng của GTSAM cho xử lý point clouds và 3D mapping.

**Cài đặt gtsam_points:**

```bash
# Clone gtsam_points repository
git clone https://github.com/hungvuhust/gtsam_points

# Về ros2_ws
cd ~/ros2_ws

# Cấu hình với CMake và build với colcon 
colcon build --packages-select gtsam_points \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
```

**Các tùy chọn CMake cho gtsam_points:**
```bash
# Các tham số tùy chọn cho cmake
cmake .. \
  -DBUILD_DEMO=OFF \                # Set ON để build demo programs
  -DBUILD_TESTS=OFF \               # Set ON để build unit tests
  -DBUILD_TOOLS=OFF \               # Set ON để build tools
  -DBUILD_WITH_TBB=OFF \            # Set ON để enable TBB
  -DBUILD_WITH_OPENMP=ON \          # Set ON để enable OpenMP (Default)
  -DBUILD_WITH_CUDA=OFF \           # Set ON để enable CUDA support
  -DBUILD_WITH_CUDA_MULTIARCH=OFF \ # Set ON để enable multi-arch CUDA support
  -DCMAKE_CUDA_ARCHITECTURES=89 \   # Nếu không chỉ định, sẽ dùng "native" architecture
  -DBUILD_WITH_MARCH_NATIVE=OFF     # Set ON để enable -march=native (khuyến nghị giữ OFF)
```

#### 3. GLIM (3D Localization and Mapping Framework)
GLIM là framework đa năng và mở rộng cho 3D localization và mapping dựa trên point cloud. Framework này cung cấp các module ước lượng khác nhau để phù hợp với nhiều kịch bản sử dụng, từ mapping chính xác với GPU đến mapping real-time nhẹ trên PC cấu hình thấp như Raspberry Pi.

**Cài đặt GLIM:**

```bash
# Clone GLIM repository
git clone https://github.com/hungvuhust/glim.git --recurse-submodules

# Về ros2_ws
cd ~/ros2_ws

# Cấu hình với CMake và build với colcon 
colcon build --packages-select glim \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```


#### 4. glim_ros2 (ROS2 Interface for GLIM)
glim_ros2 là package ROS2 cung cấp interface để sử dụng GLIM framework trong môi trường ROS2. Package này bao gồm các node và tools để tích hợp GLIM với ROS2 ecosystem.

**Cài đặt glim_ros2:**

```bash
# Clone glim_ros2 repository
git clone https://github.com/hungvuhust/glim_ros2.git

# Về ros2_ws
cd ~/ros2_ws

# Cấu hình với CMake và build với colcon 
colcon build --packages-select glim_ros \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
