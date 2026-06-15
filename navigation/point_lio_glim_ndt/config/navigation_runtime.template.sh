#!/usr/bin/env bash

# Public runtime template for the paper navigation artifacts.
# Copy this file to navigation_runtime.local.sh and edit values for a local lab.

set -euo pipefail

export NAV_RUNTIME_CONTAINER_NAME="${NAV_RUNTIME_CONTAINER_NAME:-navigation_runtime}"
export NAV_RUNTIME_IMAGE_TAR_FILE="${NAV_RUNTIME_IMAGE_TAR_FILE:-navigation_runtime_image.tar.gz}"
export NAV_RUNTIME_DATA_VOLUME="${NAV_RUNTIME_DATA_VOLUME:-navigation_runtime_data}"

export ROBOT_PROFILE="${ROBOT_PROFILE:-humanoid_mid360}"
export ROBOT_NAME="${ROBOT_NAME:-humanoid_robot}"
export LIDAR_TYPE="${LIDAR_TYPE:-3D}"

export SENSOR_DRIVER_LOCATION="${SENSOR_DRIVER_LOCATION:-container}"
export HOST_LIVOX_DRIVER_WS="${HOST_LIVOX_DRIVER_WS:-/path/to/livox_driver_ws}"
export LIVOX_DRIVER_WS_IN_CONTAINER="${LIVOX_DRIVER_WS_IN_CONTAINER:-/opt/livox_driver_ws}"
export LIVOX_DRIVER_CONFIG_IN_CONTAINER="${LIVOX_DRIVER_CONFIG_IN_CONTAINER:-${LIVOX_DRIVER_WS_IN_CONTAINER}/src/livox_ros_driver2/config}"

export RUNTIME_ROOT_IN_CONTAINER="${RUNTIME_ROOT_IN_CONTAINER:-/opt/navigation_runtime}"
export SLAM_ROOT_IN_CONTAINER="${SLAM_ROOT_IN_CONTAINER:-${RUNTIME_ROOT_IN_CONTAINER}/slam}"
export NAV_ROOT_IN_CONTAINER="${NAV_ROOT_IN_CONTAINER:-${RUNTIME_ROOT_IN_CONTAINER}/navigation}"

export SLAM_USE_RVIZ="${SLAM_USE_RVIZ:-false}"
export SLAM_ROBOT_TYPE="${SLAM_ROBOT_TYPE:-legged}"
export SLAM_CONFIG_NAME="${SLAM_CONFIG_NAME:-point_lio_glim_ndt_mid360}"
export SLAM_CONFIG_FILE="${SLAM_CONFIG_FILE:-${SLAM_ROOT_IN_CONTAINER}/config/${SLAM_CONFIG_NAME}.yaml}"
export SUBMAP3D_BASE_DIR="${SUBMAP3D_BASE_DIR:-${SLAM_ROOT_IN_CONTAINER}/data/submap3D}"

export NAV_USE_RVIZ="${NAV_USE_RVIZ:-false}"
export NAV_ODOM_SOURCE="${NAV_ODOM_SOURCE:-robot}"
export NAV_TYPE="${NAV_TYPE:-straight}"
export USE_OBSTACLE_DETECTION="${USE_OBSTACLE_DETECTION:-true}"
export USE_NAV_2D_MAPPING="${USE_NAV_2D_MAPPING:-true}"
export GRID_MAP_2D_NAME="${GRID_MAP_2D_NAME:-navigation_grid_map}"

ensure_container_running() {
    local container_name="$1"

    if docker inspect "$container_name" >/dev/null 2>&1; then
        if [ "$(docker inspect -f '{{.State.Running}}' "$container_name")" != "true" ]; then
            docker start "$container_name" >/dev/null
        fi
        return 0
    fi

    echo "Container '$container_name' does not exist. Create it from the local runtime image before running this script." >&2
    return 1
}

sync_display_if_available() {
    local container_name="$1"

    if [ -z "${DISPLAY:-}" ]; then
        export SLAM_USE_RVIZ=false
        export NAV_USE_RVIZ=false
        return 0
    fi

    xhost +local:docker >/dev/null 2>&1 || true
    docker exec "$container_name" bash -lc "export DISPLAY=${DISPLAY}" >/dev/null 2>&1 || true
}
