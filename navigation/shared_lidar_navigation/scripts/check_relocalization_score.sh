#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODULE_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CONFIG_FILE="${NAVIGATION_RUNTIME_CONFIG:-$MODULE_ROOT/config/navigation_runtime.template.sh}"

source "$CONFIG_FILE"

ensure_container_running "$NAV_RUNTIME_CONTAINER_NAME"

docker exec "$NAV_RUNTIME_CONTAINER_NAME" bash -lc "
    set -e

    latest_dir=\$(ls -1dt ${SUBMAP3D_BASE_DIR}/*/ 2>/dev/null | head -n 1)
    if [ -z \"\$latest_dir\" ]; then
        echo '[ERROR] No submap directory found' >&2
        exit 10
    fi

    result_file=\"\${latest_dir}/matchResult/result\"
    if [ ! -f \"\$result_file\" ]; then
        echo \"[ERROR] Result file not found: \$result_file\" >&2
        exit 11
    fi

    result_score=\$(grep -E '^\\s*result_score\\s*:' \"\$result_file\" | awk -F':' '{print \$2}' | tr -d ' ')
    b_converge=\$(grep -E '^\\s*b_converge\\s*:' \"\$result_file\" | awk -F':' '{print \$2}' | tr -d ' ')
    threshold=\$(grep -E '^\\s*LocalLoc3D\\.submapMatcher\\.reloc_align_score_th\\s*:' \"${SLAM_CONFIG_FILE}\" | sed 's/#.*//' | awk -F':' '{print \$2}' | tr -d ' ')

    if [ -z \"\$result_score\" ] || [ -z \"\$b_converge\" ] || [ -z \"\$threshold\" ]; then
        echo '[ERROR] Relocalization score, convergence flag, or threshold is missing.' >&2
        exit 12
    fi

    echo \"latest_submap: \$(basename \"\$latest_dir\")\"
    echo \"b_converge: \$b_converge\"
    echo \"result_score: \$result_score\"
    echo \"reloc_threshold: \$threshold\"

    awk -v score=\"\$result_score\" -v threshold=\"\$threshold\" 'BEGIN {
        if (score >= threshold) exit 0
        exit 20
    }'
"
