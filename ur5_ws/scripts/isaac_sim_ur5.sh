#!/bin/bash -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

"${ISAACSIM_PYTHON_EXE}" "${SCRIPT_DIR}/isaac/spawn_ur.py" --ur_type ur5 "$@"
