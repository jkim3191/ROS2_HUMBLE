# Complete ROS2_HUMBLE Workspace Docker Configuration
# Supports both elevation_mapping_cupy and digit_mujoco packages

# Use the existing elevation mapping Docker as base
FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04 AS base

# Build arguments
ARG ROS_DISTRO=humble
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG RMW_NAME=fastrtps
ARG INSTALL_EMCUPY_ROSDEPS=true

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=${LANG}\
    TZ=UTC \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_ROOT=/opt/ros/${ROS_DISTRO} \
    AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:/usr/local/cuda/lib64 \
    PATH=/opt/ros/${ROS_DISTRO}/bin:/usr/src/tensorrt/bin:/usr/local/cuda/bin:$PATH \
    PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages \
    RMW_IMPLEMENTATION=rmw_${RMW_NAME}_cpp \
    # TurtleBot3 simulation support
    TURTLEBOT3_MODEL=waffle_realsense_depth \
    # MuJoCo environment
    MUJOCO_GL=egl

# Install basic utilities and dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
        locales \
        tzdata \
        curl \
        gnupg2 \
        lsb-release \
        sudo \
        software-properties-common \
        wget \
        git \
        git-lfs \
        nano \
        vim \
        htop \
        tree \
        build-essential \
        cmake \
        pkg-config \
        # Graphics and GUI support
        x11-apps \
        mesa-utils \
        libgl1-mesa-glx \
        libglu1-mesa-dev \
        # MuJoCo specific dependencies
        libglfw3-dev \
        libosmesa6-dev \
        libglew-dev \
        # Python development
        python3 \
        python3-pip \
        python3-dev \
        python3-numpy \
        python3-scipy \
        python3-matplotlib \
        # Additional utilities
        tmux \
        screen \
    && locale-gen ${LANG} \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 repository and install ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop-full \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-rmw-cyclonedx-cpp \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-vcstool \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true

# Install elevation_mapping_cupy specific ROS2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt update && apt install -y --no-install-recommends \
        # Core dependencies
        libboost-all-dev \
        # Grid map packages
        ros-${ROS_DISTRO}-grid-map-msgs \
        ros-${ROS_DISTRO}-grid-map-ros \
        ros-${ROS_DISTRO}-grid-map-cv \
        ros-${ROS_DISTRO}-grid-map-core \
        ros-${ROS_DISTRO}-grid-map-demos \
        # Sensor and image processing
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-point-cloud-transport \
        # Transformations and visualization
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-rviz2 \
        # Simulation
        ros-${ROS_DISTRO}-gazebo-ros \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        # TurtleBot3 simulation support
        ros-${ROS_DISTRO}-turtlebot3 \
        ros-${ROS_DISTRO}-turtlebot3-msgs \
        ros-${ROS_DISTRO}-turtlebot3-simulations \
        ros-${ROS_DISTRO}-turtlebot3-teleop \
        # Additional Python packages
        python3-shapely \
        python3-ruamel.yaml \
        python3-opencv \
        # Build tools
        python3-pybind11 \
        pybind11-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for elevation_mapping_cupy
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install \
        # Core scientific computing
        "numpy<2.0.0" \
        scipy \
        matplotlib \
        opencv-python \
        transforms3d \
        # ROS2 utilities
        rosdoc2 \
        colcon-clean \
        # GPU acceleration
        cupy-cuda12x \
        # Configuration and parsing
        simple-parsing \
        ruamel.yaml \
        # Development tools
        autopep8 \
        flake8-builtins \
        flake8-comprehensions \
        flake8-docstrings \
        flake8-import-order \
        flake8-class-newline \
        flake8-blind-except \
        flake8-quotes \
        # Documentation
        sphinx_rtd_theme \
        sphinx-multiversion \
        sphinx-copybutton \
        sphinx-tabs

# Install MuJoCo and digit_mujoco specific dependencies
RUN python3 -m pip install \
        # MuJoCo simulation
        mujoco \
        mujoco-python-viewer \
        # Machine learning and optimization
        scikit-learn \
        pandas \
        # Visualization and analysis
        pygame \
        moviepy \
        imageio \
        # Development utilities
        click \
        psutil \
        setproctitle \
        cma \
        # Garage dependencies
        akro \
        dowel \
        # Distributed computing
        ray \
        # Jupyter for notebooks
        jupyter \
        jupyterlab \
        notebook

# Set up non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up user environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:/workspace/install/lib/python3.10/site-packages" >> /home/$USERNAME/.bashrc \
    && echo "export TURTLEBOT3_MODEL=waffle_realsense_depth" >> /home/$USERNAME/.bashrc \
    && echo "alias cb='colcon build --symlink-install'" >> /home/$USERNAME/.bashrc \
    && echo "alias cbs='colcon build --symlink-install --packages-select'" >> /home/$USERNAME/.bashrc \
    && echo "alias ws='cd /workspace && source install/setup.bash'" >> /home/$USERNAME/.bashrc

# Set up rosdep for the user
USER $USERNAME
RUN rosdep update

# Switch back to root for final setup
USER root

# Install NVIDIA Container Toolkit for GPU support
RUN distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
    && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add - \
    && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list \
    && apt update && apt install -y nvidia-container-toolkit \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory structure
RUN mkdir -p /workspace \
    && chown -R $USERNAME:$USERNAME /workspace

# Set working directory
WORKDIR /workspace

# Switch to user
USER $USERNAME

# Default command
CMD ["/bin/bash"]