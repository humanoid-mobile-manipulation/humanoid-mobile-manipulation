#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODULE_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CONFIG_FILE="${NAVIGATION_RUNTIME_CONFIG:-$MODULE_ROOT/config/navigation_runtime.template.sh}"

source "$CONFIG_FILE"

ensure_container_running "$NAV_RUNTIME_CONTAINER_NAME"
sync_display_if_available "$NAV_RUNTIME_CONTAINER_NAME"

docker exec -it "$NAV_RUNTIME_CONTAINER_NAME" bash -lc "
    set -e
    cd ${SLAM_ROOT_IN_CONTAINER}
    bash start_slam.sh ${SLAM_USE_RVIZ} ${SLAM_ROBOT_TYPE} ${SLAM_CONFIG_NAME}
"
