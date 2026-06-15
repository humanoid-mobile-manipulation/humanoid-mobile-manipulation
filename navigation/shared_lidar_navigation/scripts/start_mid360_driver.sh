#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODULE_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CONFIG_FILE="${NAVIGATION_RUNTIME_CONFIG:-$MODULE_ROOT/config/navigation_runtime.template.sh}"
MID360_CONFIG="${MID360_CONFIG:-$MODULE_ROOT/config/mid360_sensor_config.template.json}"

source "$CONFIG_FILE"

if [ "$SENSOR_DRIVER_LOCATION" = "container" ]; then
    ensure_container_running "$NAV_RUNTIME_CONTAINER_NAME"
    docker cp "$MID360_CONFIG" "$NAV_RUNTIME_CONTAINER_NAME:$LIVOX_DRIVER_CONFIG_IN_CONTAINER/MID360_config.json"
    docker exec -it "$NAV_RUNTIME_CONTAINER_NAME" bash -lc "
        set -e
        source ${LIVOX_DRIVER_WS_IN_CONTAINER}/devel/setup.bash
        roslaunch livox_ros_driver2 rviz_MID360.launch rviz_enable:=false
    "
elif [ "$SENSOR_DRIVER_LOCATION" = "host" ]; then
    if [ ! -d "$HOST_LIVOX_DRIVER_WS" ]; then
        echo "HOST_LIVOX_DRIVER_WS does not exist: $HOST_LIVOX_DRIVER_WS" >&2
        exit 1
    fi

    cp "$MID360_CONFIG" "$HOST_LIVOX_DRIVER_WS/src/livox_ros_driver2/config/MID360_config.json"
    source /opt/ros/noetic/setup.bash
    source "$HOST_LIVOX_DRIVER_WS/devel/setup.bash"
    roslaunch livox_ros_driver2 rviz_MID360.launch rviz_enable:=false
else
    echo "SENSOR_DRIVER_LOCATION must be 'container' or 'host'." >&2
    exit 1
fi
