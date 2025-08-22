#!/bin/bash

# Set ROS distribution
ROS_DISTRO="humble"

# Set up XDG runtime directory for GUI apps
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Set the ROS workspace path
export ROS_WS=/root/ros2_ws # Using the path from our previous examples

# --- (Optional but good) ROS_DOMAIN_ID logic ---
# For this to be useful, you should mount a shared volume to /root/shared
# SHARED_DIR="/root/shared"
# mkdir -p $SHARED_DIR
# ROS_DOMAIN_ID_FILE="$SHARED_DIR/ros_domain_id.txt"
# if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
#     echo "0" > "$ROS_DOMAIN_ID_FILE"
# fi
# export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")
# ----------------------------------------------------

# Source the main ROS setup file
source /opt/ros/$ROS_DISTRO/setup.bash

# Source the local workspace setup file, if it exists
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
fi

# Execute the command passed to the container
exec "$@"