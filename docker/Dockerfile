# ====================================================================================
# FINAL, PROVEN DOCKERFILE for a High-Performance Autonomous Vehicle
# This version starts from a pre-built base and adds your custom dependencies.
# ====================================================================================

# --- Stage 1: The Foundation ---
# START from a pre-built, high-performance base image. This image ALREADY CONTAINS:
#  - A correct L4T r36.3 base OS
#  - The full NVIDIA JetPack SDK (CUDA, cuDNN, TensorRT)
#  - GPU-accelerated PyTorch and Torchvision (if you use the 'pytorch' tag)
#  - ROS 2 Humble (ros-base)
# We will use the 'ros-base' variant for a leaner image.
ARG L4T_VERSION=r36.3.0
FROM dustynv/ros:humble-ros-base-l4t-${L4T_VERSION}


# --- Stage 2: FIX - Update the Expired ROS 2 GPG Key ---

# Step 2.1: Install the necessary tools (curl, gnupg2) first.
# We temporarily disable the ROS 2 sources list to bypass the key error for this step.
RUN mv /etc/apt/sources.list.d/ros2.list /tmp/ && \
    apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg2 && \
    mv /tmp/ros2.list /etc/apt/sources.list.d/

# Step 2.2: Now that we have curl, download and install the new, valid key.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


# --- Stage 3: Install YOUR Project's System Dependencies ---
# Now that the key is fixed, all subsequent 'apt-get update' calls will succeed.
RUN apt-get update && apt-get install -y --no-install-recommends \
    # For CAN Bus communication
    can-utils \
    # For the ROS 2 Navigation Stack (Nav2)
    
    # For manual control/debugging
    ros-humble-teleop-twist-keyboard \
    # For plotting and visualization
    python3-matplotlib \
    python3-seaborn \
    python3-pygame \
    graphviz \
    && rm -rf /var/lib/apt/lists

# --- Stage 4: Add YOUR Project's Python Dependencies ---
# The base image already has numpy, torch, and torchvision.
# --- Stage 4: Add YOUR Project's Python Dependencies ---
# The base image already has numpy, torch, and torchvision.
#
# IMPORTANT FIX: Add '--ignore-installed' to prevent conflicts with
# system-level packages (like sympy) that were installed by apt.
# This tells pip to install its own versions without trying to uninstall the system's.
#
RUN pip3 install --no-cache-dir \
    --index-url https://pypi.org/simple \
    --trusted-host pypi.org \
    --ignore-installed \
    # For CAN Bus interfacing in Python
    python-can \
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
    pyyaml \
    # For advanced ML model serving (from inventory)
    tritonclient[all] \
    # For JSON schema validation (from inventory)
    jsonschema \
    jsonpatch \
    jsonpointer
    
# --- Stage 5: Set Up and Build the ROS Workspace ---
ENV ROS_WS=/root/ros2_ws
WORKDIR ${ROS_WS}

# Copy your ROS 2 source code into the container.
COPY ./src ./src

# Build your ROS 2 workspace.
RUN . /opt/ros/humble/setup.bash && colcon build


# --- Stage 6: Final Configuration ---
# Automatically source your workspace every time a new terminal is opened.
RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc