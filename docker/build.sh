#!/bin/bash

# This script is designed to be run from within the 'docker' directory.
# It will automatically find the workspace root and use it as the build context.

# --- CONFIGURATION ---
IMAGE_NAME="jetson-robot:humble"
# ---------------------

# --- SCRIPT LOGIC ---

# Get the absolute path of the directory where THIS script is located.
# This will correctly resolve to ~/review_ws/docker
SCRIPT_DIR=$(dirname $(realpath "$0"))

# The ROS 2 workspace root is the PARENT directory of this script's location.
# This will correctly resolve to ~/review_ws
WORKSPACE_ROOT=$(realpath "$SCRIPT_DIR/../")

# The Dockerfile is located in the same directory as this script.
DOCKERFILE_PATH="${SCRIPT_DIR}/Dockerfile"

# --- Pre-flight checks to prevent errors ---
echo "--- Verifying file structure ---"
if [ ! -d "${WORKSPACE_ROOT}/src" ]; then
    echo "ERROR: 'src' directory not found in workspace root: ${WORKSPACE_ROOT}"
    exit 1
fi
if [ ! -f "${DOCKERFILE_PATH}" ]; then
    echo "ERROR: 'Dockerfile' not found at expected location: ${DOCKERFILE_PATH}"
    exit 1
fi
echo "File structure verified."
echo ""

# --- Build the image ---
echo "--- Building Docker image '${IMAGE_NAME}' ---"
echo "Using Dockerfile: ${DOCKERFILE_PATH}"
echo "Using Build Context: ${WORKSPACE_ROOT}"
echo ""

# This is the corrected docker build command:
# -f : Explicitly sets the path to the Dockerfile.
# -t : Sets the tag (name) of the image.
# The final argument is the build context path. It MUST be the workspace root.
sudo docker image build \
    -f "${DOCKERFILE_PATH}" \
    -t "${IMAGE_NAME}" \
    "${WORKSPACE_ROOT}"

# --- Report status ---
if [ $? -eq 0 ]; then
    echo ""
    echo "--- Build successful. Image '${IMAGE_NAME}' is ready. ---"
else
    echo ""
    echo "--- !!! BUILD FAILED. Please check the Docker errors above. !!! ---"
fi