#!/bin/bash
# Post-creation setup script for ROS2_HUMBLE workspace
# Configures both elevation_mapping_cupy and digit_mujoco packages

set -e

echo "🚀 Setting up ROS2_HUMBLE workspace for elevation mapping and digit simulation..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /workspace

# Update rosdep
echo "📦 Updating rosdep database..."
rosdep update

# Install workspace dependencies
echo "🔧 Installing workspace dependencies..."
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true

# Set up digit_mujoco Python bindings
echo "🤖 Setting up digit_mujoco Python bindings..."
if [ -d "src/digit_mujoco/src/digit_mujoco" ]; then
    cd src/digit_mujoco/src/digit_mujoco
    
    # Update the path in DigitControlPybind for container environment
    if [ -f "DigitControlPybind/DigitControlPybind.cpp" ]; then
        echo "Updating DigitControlPybind path for container environment..."
        # This would need to be updated based on the actual file structure
        # For now, just ensure the directory exists
        mkdir -p DigitControlPybind/src/include/digit_controller/src
    fi
    
    # Install digit_mujoco Python package
    echo "Installing digit_mujoco Python package..."
    pip install -e . || echo "⚠️  digit_mujoco installation may need manual configuration"
    
    cd /workspace
fi

# Build the workspace
echo "🔨 Building ROS2 workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Source the built workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace built and sourced successfully!"
else
    echo "⚠️  Build may have failed, check logs"
fi

# Set up useful aliases and environment
echo "⚙️  Setting up development environment..."

# Add workspace-specific aliases to .bashrc if not already present
if ! grep -q "# ROS2_HUMBLE workspace aliases" ~/.bashrc; then
    cat >> ~/.bashrc << 'EOF'

# ROS2_HUMBLE workspace aliases
alias ws='cd /workspace && source install/setup.bash'
alias cb='colcon build --symlink-install'
alias cbs='colcon build --symlink-install --packages-select'
alias cbt='colcon test --packages-select'
alias cbr='colcon build --symlink-install --packages-select --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'
alias src='source install/setup.bash'
alias em='ros2 launch elevation_mapping_cupy'
alias digit='cd /workspace/src/digit_mujoco && python3 src/digit_mujoco/test/digit_main.py'

# Useful ROS2 aliases
alias rqt='rqt --force-discover'
alias rviz='rviz2'
alias topics='ros2 topic list'
alias nodes='ros2 node list'
alias tf='ros2 run tf2_tools view_frames'

# Git shortcuts
alias gs='git status'
alias ga='git add'
alias gc='git commit'
alias gp='git push'
alias gl='git log --oneline -10'

EOF
fi

# Create useful directories
mkdir -p /workspace/logs
mkdir -p /workspace/data
mkdir -p /workspace/config

# Set up environment variables for the session
export TURTLEBOT3_MODEL=waffle_realsense_depth
export ROS_DOMAIN_ID=0
export PYTHONPATH=$PYTHONPATH:/workspace/install/lib/python3.10/site-packages

echo "🎉 Setup complete! Your ROS2_HUMBLE workspace is ready for development."
echo ""
echo "📋 Quick start commands:"
echo "  ws           - Navigate to workspace and source environment"
echo "  cb           - Build entire workspace"
echo "  cbs <pkg>    - Build specific package"
echo "  em <launch>  - Launch elevation mapping configurations"
echo "  digit        - Run digit MuJoCo simulation"
echo ""
echo "🧪 Test your setup:"
echo "  1. Elevation mapping: ros2 launch elevation_mapping_cupy elevation_mapping_cupy.launch.py"
echo "  2. TurtleBot3 sim:    ros2 launch elevation_mapping_cupy turtlesim_init.launch.py"
echo "  3. Digit simulation:  cd src/digit_mujoco && python3 src/digit_mujoco/test/digit_main.py"
echo ""
echo "🔍 Check package status:"
echo "  colcon list"
echo "  ros2 pkg list | grep -E '(elevation|digit)'"
echo ""