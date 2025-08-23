#!/bin/bash

# This script is designed to be run from within the 'docker' directory.
# It will automatically find the workspace root and use it as the build context.

# --- CONFIGURATION ---
# The name for your final, custom image.
IMAGE_NAME="jetson-abaja"

# The L4T version that matches your Dockerfile's ARG.
# This should match your Jetson's OS version.
L4T_VERSION="r36.3.0"
# ---------------------

# --- SCRIPT LOGIC ---
set -e # Exit immediately if a command exits with a non-zero status.

# Get the absolute path of the directory where THIS script is located.
SCRIPT_DIR=$(dirname $(realpath "$0"))

# The ROS 2 workspace root is the PARENT directory of this script's location.
WORKSPACE_ROOT=$(realpath "$SCRIPT_DIR/../")

# The Dockerfile is located in the same directory as this script.
DOCKERFILE_PATH="${SCRIPT_DIR}/Dockerfile"

# --- Pre-flight checks to prevent errors ---
echo "--- Verifying file structure ---"
if [ ! -d "${WORKSPACE_ROOT}/src" ]; then
    # Changed from an error to a warning to allow building a base image without code.
    echo "WARNING: 'src' directory not found in workspace root: ${WORKSPACE_ROOT}"
    echo "         Build will continue, but the COPY step in the Dockerfile might fail if it's uncommented."
fi
if [ ! -f "${DOCKERFILE_PATH}" ]; then
    echo "ERROR: 'Dockerfile' not found at expected location: ${DOCKERFILE_PATH}"
    exit 1
fi
echo "File structure verified."
echo ""

# --- Build the image ---
echo "--- Building Docker image '${IMAGE_NAME}' ---"
echo "Using Dockerfile:   ${DOCKERFILE_PATH}"
echo "Using Build Context:  ${WORKSPACE_ROOT}"
echo "Using L4T Version:    ${L4T_VERSION}"
echo ""

# -f : Explicitly sets the path to the Dockerfile.
# -t : Sets the tag (name) of the image.
# --build-arg : Passes the L4T_VERSION into the Dockerfile.
# The final argument is the build context path. It MUST be the workspace root.
sudo docker image build \
    -f "${DOCKERFILE_PATH}" \
    -t "${IMAGE_NAME}" \
    --build-arg L4T_VERSION=${L4T_VERSION} \
    "${WORKSPACE_ROOT}"

# --- Report status ---
# The 'set -e' at the top handles the failure case, so we only need a success message.
echo ""
echo "--- Build successful. Image '${IMAGE_NAME}' is ready. ---"