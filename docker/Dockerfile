# ====================================================================================
# DOCKERFILE: Building a Custom, High-Performance ROS 2 Humble Image from Scratch
# ====================================================================================

# --- Stage 1: The Foundation (The "Scratch" OS) ---
# Use the correct, minimal L4T base image for your Jetson's OS version.
# This contains the basic OS and hooks for the NVIDIA Container Runtime.
ARG L4T_VERSION=r36.3.0
FROM nvcr.io/nvidia/l4t-base:${L4T_VERSION}


# --- Stage 2: Basic Housekeeping ---
# Set DEBIAN_FRONTEND to prevent apt-get from hanging on interactive prompts.
ENV DEBIAN_FRONTEND=noninteractive

# Set up the correct UTF-8 locale to prevent errors with some tools.
RUN apt-get update && apt-get install -y --no-install-recommends locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*


# --- Stage 3: Install the NVIDIA JetPack SDK ---
# This is the most critical step to unlock the GPU. We install the same SDK
# that would be on your host Jetson, but inside the container.
# This provides CUDA, cuDNN, TensorRT, VPI, and hardware-accelerated GStreamer/OpenCV.
RUN apt-get update && apt-get install -y --no-install-recommends nvidia-jetpack && \
    rm -rf /var/lib/apt/lists/*


# --- Stage 4: Install a GPU-enabled PyTorch ---
# We must install a specific, pre-built version of PyTorch for Jetson.
# Using a generic 'pip install torch' will NOT provide GPU support.
RUN apt-get update && apt-get install -y --no-install-recommends libopenblas-dev python3-pip && \
    pip3 install --no-cache-dir numpy && \
    pip3 install --no-cache-dir --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v60 torch==2.3.0a0+ebed864214.240513 torchvision==0.18.0a0+a772b1c --upgrade


# --- Stage 5: Add the Official ROS 2 Software Repository ---
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    rm -rf /var/lib/apt/lists/*


# --- Stage 6: Install ROS 2 and Your Project's System Dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Core ROS 2 installation
    ros-humble-ros-base \
    # Essential ROS 2 development tools
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    # For CAN Bus communication
    can-utils \
    # For the ROS 2 Navigation Stack (Nav2)
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # For manual control/debugging
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*


# --- Stage 7: Add Your Project's Python Dependencies ---
RUN pip3 install --no-cache-dir \
    # For CAN Bus interfacing in Python
    python-can \
    cantools \
    # For running YOLO models for perception
    ultralytics \
    # For data analysis and manipulation
    pandas \
    scipy \
    # For serial communication (GPS, IMU, etc.)
    pyserial \
    # For hardware that needs spidev
    spidev \
    # For handling YAML configuration files
    pyyaml


# --- Stage 8: Set Up and Build the ROS Workspace ---
ENV ROS_WS=/root/ros2_ws
WORKDIR ${ROS_WS}

# Copy your (currently empty) src folder. This is good practice.
COPY ./src ./src

# Initialize rosdep to install dependencies for ROS packages from source.
RUN rosdep init && rosdep update

# Install dependencies for any code you add to the 'src' folder.
RUN . /opt/ros/humble/setup.bash && rosdep install -y \
      --from-paths src \
      --ignore-src \
      --rosdistro humble \
      -r || true # Use '|| true' to prevent failure if src is empty and rosdep finds no packages

# Build the workspace. If src is empty, this command will do nothing but will succeed.
RUN . /opt/ros/humble/setup.bash && colcon build


# --- Stage 9: Final Configuration ---
# Automatically source the ROS 2 environment and your workspace when a terminal starts.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc