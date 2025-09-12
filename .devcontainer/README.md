# ROS2_HUMBLE DevContainer Setup

This DevContainer configuration provides a complete development environment for the ROS2_HUMBLE workspace, supporting both **elevation_mapping_cupy** and **digit_mujoco** packages.

## 🚀 Features

### Core Capabilities
- **ROS2 Humble** with full desktop installation
- **CUDA 12.1** support for GPU acceleration
- **MuJoCo** physics simulation
- **TurtleBot3** simulation environment
- **Complete toolchain** for C++ and Python development

### Package Support
- ✅ **elevation_mapping_cupy**: GPU-accelerated elevation mapping
- ✅ **digit_mujoco**: Humanoid robot simulation with MuJoCo
- ✅ **semantic_sensor**: Multi-modal sensor processing
- ✅ **plane_segmentation**: Terrain analysis suite
- ✅ **All message packages**: Custom ROS2 message definitions

## 📋 Prerequisites

### Host System Requirements
1. **Docker** with BuildKit support
2. **NVIDIA Docker** (nvidia-docker2) for GPU support
3. **VS Code** with Remote-Containers extension
4. **NVIDIA GPU** with CUDA 12.x compatible drivers

### Installation Commands
```bash
# Install Docker (Ubuntu/Debian)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install NVIDIA Docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Verify GPU support
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu22.04 nvidia-smi
```

## 🛠️ Usage

### Opening the DevContainer

1. **Open in VS Code**:
   ```bash
   cd /path/to/ROS2_HUMBLE
   code .
   ```

2. **Start DevContainer**:
   - Press `Ctrl+Shift+P`
   - Type "Remote-Containers: Reopen in Container"
   - Select the command and wait for container build

3. **Alternative Command Line**:
   ```bash
   # From the workspace root
   docker build -t ros2-humble-workspace .devcontainer
   docker run -it --gpus all --name ros2-dev ros2-humble-workspace
   ```

### First-Time Setup

The container will automatically run the setup script, but you can also run it manually:

```bash
# Inside the container
cd /workspace
./.devcontainer/setup.sh
```

## 🧪 Testing Your Setup

### 1. Verify ROS2 Installation
```bash
# Check ROS2 environment
ros2 --version
ros2 topic list

# Verify packages
ros2 pkg list | grep -E "(elevation|digit)"
colcon list
```

### 2. Test Elevation Mapping
```bash
# Basic elevation mapping
ros2 launch elevation_mapping_cupy elevation_mapping_cupy.launch.py

# TurtleBot3 simulation with elevation mapping
export TURTLEBOT3_MODEL=waffle_realsense_depth
ros2 launch elevation_mapping_cupy turtlesim_init.launch.py
```

### 3. Test Digit MuJoCo Simulation
```bash
# Navigate to digit package
cd src/digit_mujoco/src/digit_mujoco

# Configure simulation mode in test/digit_main.py
# Set PSP_flag = True for demo, False for manual control

# Run simulation
python3 test/digit_main.py
```

### 4. Test GPU Acceleration
```bash
# Check CUDA availability
python3 -c "import cupy; print('CuPy version:', cupy.__version__)"
nvidia-smi

# Test elevation mapping GPU kernels
cd src/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/tests
pytest test_custom_kernels.py -v
```

## 🔧 Development Workflow

### Building Packages
```bash
# Build entire workspace
cb  # alias for 'colcon build --symlink-install'

# Build specific package
cbs elevation_mapping_cupy
cbs digit_mujoco

# Build with debug symbols
cbr elevation_mapping_cupy  # RelWithDebInfo build
```

### Testing
```bash
# Run Python tests
cd src/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/tests
pytest

# Test specific functionality
cbt elevation_mapping_cupy  # colcon test
```

### Useful Aliases
The setup script creates helpful aliases:
- `ws` - Navigate to workspace and source environment
- `cb` - Build entire workspace
- `cbs <pkg>` - Build specific package
- `em <launch>` - Launch elevation mapping configurations
- `digit` - Run digit simulation
- `topics` - List ROS2 topics
- `nodes` - List ROS2 nodes

## 📁 Workspace Structure

```
/workspace/
├── .devcontainer/          # DevContainer configuration
│   ├── devcontainer.json   # VS Code container config
│   ├── Dockerfile         # Container image definition
│   ├── setup.sh          # Post-creation setup script
│   └── README.md         # This file
├── src/                  # Source packages
│   ├── elevation_mapping_cupy/  # GPU elevation mapping
│   └── digit_mujoco/           # Humanoid robot simulation
├── build/               # Build artifacts
├── install/            # Installation directory
├── log/               # Build and runtime logs
└── CURSOR.md         # Comprehensive workspace documentation
```

## 🐛 Troubleshooting

### Common Issues

1. **GPU Not Available**
   ```bash
   # Check NVIDIA drivers
   nvidia-smi
   
   # Verify Docker GPU support
   docker run --rm --gpus all nvidia/cuda:12.1-base nvidia-smi
   ```

2. **Build Failures**
   ```bash
   # Clean and rebuild
   colcon build --symlink-install --cmake-clean-cache
   
   # Check dependencies
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Python Import Errors**
   ```bash
   # Check Python path
   echo $PYTHONPATH
   
   # Source workspace
   source install/setup.bash
   ```

4. **Display Issues (GUI Applications)**
   ```bash
   # Check X11 forwarding
   echo $DISPLAY
   xhost +local:docker  # On host system
   ```

### Package-Specific Issues

**Elevation Mapping**:
- Ensure CuPy is installed: `python3 -c "import cupy"`
- Check CUDA version compatibility
- Verify NumPy version < 2.0.0

**Digit MuJoCo**:
- Update file paths in `DigitControlPybind.cpp` if needed
- Ensure MuJoCo license if using Pro features
- Check Python bindings with `pip list | grep mujoco`

## 🔄 Updating the Container

### Rebuilding the Container
```bash
# Remove existing container
docker container rm ros2-humble-complete-workspace

# Rebuild from VS Code
# Ctrl+Shift+P -> "Remote-Containers: Rebuild Container"
```

### Updating Dependencies
```bash
# Update Python packages
pip install --upgrade cupy-cuda12x numpy scipy

# Update ROS2 packages
sudo apt update && sudo apt upgrade
```

## 📊 Performance Optimization

### GPU Memory Management
```bash
# Monitor GPU usage
watch nvidia-smi

# CuPy memory pool settings
export CUPY_CACHE_DIR=/tmp/cupy_cache
```

### Build Performance
```bash
# Parallel compilation
export CMAKE_BUILD_PARALLEL_LEVEL=8
colcon build --parallel-workers 8
```

## 📚 Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Elevation Mapping CuPy Docs](https://leggedrobotics.github.io/elevation_mapping_cupy/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [VS Code Remote Development](https://code.visualstudio.com/docs/remote/containers)

## 🤝 Contributing

To contribute to this DevContainer setup:

1. Test changes thoroughly
2. Update this README if adding new features
3. Ensure compatibility with both packages
4. Document any new requirements or dependencies

---

**Maintainer**: Generated for ROS2_HUMBLE workspace  
**Last Updated**: September 2025