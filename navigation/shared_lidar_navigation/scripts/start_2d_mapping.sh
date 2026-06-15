#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODULE_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CONFIG_FILE="${NAVIGATION_RUNTIME_CONFIG:-$MODULE_ROOT/config/navigation_runtime.template.sh}"

source "$CONFIG_FILE"

if [ "$USE_NAV_2D_MAPPING" != "true" ]; then
    echo "USE_NAV_2D_MAPPING is not enabled; skipping 2D mapping."
    exit 0
fi

ensure_container_running "$NAV_RUNTIME_CONTAINER_NAME"
sync_display_if_available "$NAV_RUNTIME_CONTAINER_NAME"

docker exec -it "$NAV_RUNTIME_CONTAINER_NAME" bash -lc "
    set -e
    export IS_USED_IN_DOCKER_CONTAINER=true
    export NAV_USE_RVIZ=${NAV_USE_RVIZ}
    export ROBOT_PROFILE=${ROBOT_PROFILE}
    cd ${NAV_ROOT_IN_CONTAINER}
    bash start_nav_2d_mapping.sh
"
