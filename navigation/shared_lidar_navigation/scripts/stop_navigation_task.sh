#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODULE_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CONFIG_FILE="${NAVIGATION_RUNTIME_CONFIG:-$MODULE_ROOT/config/navigation_runtime.template.sh}"

source "$CONFIG_FILE"

if [ "$NAV_TYPE" = "none" ]; then
    echo "NAV_TYPE is none; no 2D navigation task is active."
    exit 0
fi

ensure_container_running "$NAV_RUNTIME_CONTAINER_NAME"

case "$NAV_TYPE" in
    move_base)
        docker exec -it "$NAV_RUNTIME_CONTAINER_NAME" bash -lc "
            set -e
            cd ${NAV_ROOT_IN_CONTAINER}/scripts/tools
            python3 stop_nav_2d_tasks.py
        "
        ;;
    straight)
        docker exec -it "$NAV_RUNTIME_CONTAINER_NAME" bash -lc "
            set -e
            source /opt/ros/noetic/setup.bash
            rosservice call /stop_nav_goal_2D '{}'
        "
        ;;
    *)
        echo "NAV_TYPE must be 'move_base', 'straight', or 'none'." >&2
        exit 1
        ;;
esac
