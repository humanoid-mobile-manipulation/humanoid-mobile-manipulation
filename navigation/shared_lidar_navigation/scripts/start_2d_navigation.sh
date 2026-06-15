#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODULE_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CONFIG_FILE="${NAVIGATION_RUNTIME_CONFIG:-$MODULE_ROOT/config/navigation_runtime.template.sh}"

source "$CONFIG_FILE"

if [ "$NAV_TYPE" = "none" ]; then
    echo "NAV_TYPE is none; 2D navigation will not start."
    exit 0
fi

ensure_container_running "$NAV_RUNTIME_CONTAINER_NAME"
sync_display_if_available "$NAV_RUNTIME_CONTAINER_NAME"

case "$NAV_TYPE" in
    move_base)
        NAV_START_SCRIPT="start_nav_2d.sh"
        ;;
    straight)
        if [ "$USE_OBSTACLE_DETECTION" = "true" ]; then
            NAV_START_SCRIPT="start_cmdVel_with_obs_det.sh"
        else
            NAV_START_SCRIPT="start_cmdVel.sh"
        fi
        ;;
    *)
        echo "NAV_TYPE must be 'move_base', 'straight', or 'none'." >&2
        exit 1
        ;;
esac

docker exec -it "$NAV_RUNTIME_CONTAINER_NAME" bash -lc "
    set -e
    export IS_USED_IN_DOCKER_CONTAINER=true
    export NAV_USE_RVIZ=${NAV_USE_RVIZ}
    export GRID_MAP_2D_NAME=${GRID_MAP_2D_NAME}
    export ROBOT_PROFILE=${ROBOT_PROFILE}
    export ROBOT_NAME=${ROBOT_NAME}
    export NAV_ODOM_SOURCE=${NAV_ODOM_SOURCE}
    cd ${NAV_ROOT_IN_CONTAINER}
    bash ${NAV_START_SCRIPT}
"
