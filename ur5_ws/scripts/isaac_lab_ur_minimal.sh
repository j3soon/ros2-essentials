#!/bin/bash -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

"${ISAACLAB_PATH}/isaaclab.sh" -p "${SCRIPT_DIR}/isaac/ur_isaaclab_minimal.py" "$@"
