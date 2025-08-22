# ====================================================================================
# FINAL RECOMMENDED DOCKERFILE
# For a ROS 2 Humble Project on Jetson (L4T r36.x / JetPack 6)
# ====================================================================================

# --- Stage 1: Define the Correct Base Image ---
# Use the L4T version for your Jetson Orin. r36.3.0 is the correct tag for JetPack 6.
ARG L4T_VERSION=r36.3.0

# Use the pre-built, multi-arch image from Dusty-NV's jetson-containers.
# This single line replaces the ~150 lines of building ROS 2 from source.
# It already includes ROS 2 Humble, CUDA-enabled OpenCV, and all dev tools.
FROM dustynv/ros:humble-ros-base-l4t-${L4T_VERSION}


# --- Stage 2: Set Up Workspace and Environment ---

# Set the ROS workspace path as a persistent environment variable.
ENV ROS_WS=/root/ros2_ws

# Set the working directory for subsequent commands.
WORKDIR ${ROS_WS}


# --- Stage 3: Add and Build YOUR Application Code ---

