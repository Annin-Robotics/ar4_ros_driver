#!/bin/bash

# Docker Configuration for 6DoF Project
# =====================================

# Default Configuration Values
TAG="latest"
NETWORK_MODE="host"
CONTAINER_NAME=""

# Argument Parsing
while [[ $# -gt 0 ]]; do
    case $1 in
        --tag)
            TAG="$2"
            shift 2
            ;;
        --network)
            NETWORK_MODE="$2"
            shift 2
            ;;
        --name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --tag TAG              Docker image tag (default: latest)"
            echo "  --network MODE         Network mode (default: host)"
            echo "  --name NAME            Container name (optional)"
            echo "  --help                 Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Docker Image Configuration
DOCKER_IMAGE="Annin-Robotics/ar4_ros_driver:$TAG"

# User and Permissions Configuration
USER_ID=$(id -u)
GROUP_ID=$(id -g)

# Hostname Configuration
HOSTNAME_VALUE=$(hostname)

# Workdir Configuration
WORKDIR=$(pwd)

# Volume Mounts Configuration
VOLUME_SPECS=(
    "/home:/home"
    "/etc/passwd:/etc/passwd:ro"
    "/etc/group:/etc/group:ro"
    "/etc/shadow:/etc/shadow:ro"
    "/tmp/.X11-unix:/tmp/.X11-unix"
)

# Build Volume Flags
VOLUME_FLAGS=()
for volume in "${VOLUME_SPECS[@]}"; do
    VOLUME_FLAGS+=("-v" "$volume")
done

# Environment Variables Configuration
ENV_VARS=(
    "DISPLAY=$DISPLAY"
    "LIBGL_ALWAYS_SOFTWARE=1"
)

# Build Environment Flags
ENV_FLAGS=()
for env_var in "${ENV_VARS[@]}"; do
    ENV_FLAGS+=("-e" "$env_var")
done

# Runtime Flags
RUNTIME_FLAGS=(
    "-it"
    "--user=$USER_ID:$GROUP_ID"
    "--group-add=sudo"
    "--hostname=$HOSTNAME_VALUE"
    "--workdir=$WORKDIR"
    "--net=host"
    "--ipc=host"
    "--rm"

)

# Add Container Name if Specified
if [ -n "$CONTAINER_NAME" ]; then
    RUNTIME_FLAGS+=("--name=$CONTAINER_NAME")
fi

# Build Docker Command
DOCKER_CMD=(
    "docker" "run"
    "${RUNTIME_FLAGS[@]}"
    "${VOLUME_FLAGS[@]}"
    "${ENV_FLAGS[@]}"
    "$DOCKER_IMAGE"
)

# Print the full command
echo "Running: ${DOCKER_CMD[@]}"
echo

# Execute the command
"${DOCKER_CMD[@]}"
    