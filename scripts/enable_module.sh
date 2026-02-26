#!/bin/bash -e

# Usage: ./scripts/enable_module.sh <MODULE_NAME>

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="${SCRIPT_DIR}/.."
TEMPLATE_COMPOSE_FILE="${REPO_ROOT}/template_ws/docker/compose.yaml"

find_current_workspace() {
    # Walk up from the current directory and return the first matching workspace.
    local dir="$PWD"
    while [ "$dir" != "/" ]; do
        if [[ "$(basename "$dir")" == *_ws ]] && [ -f "$dir/docker/compose.yaml" ]; then
            echo "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    return 1
}

resolve_workspace_dir() {
    # Resolve a workspace from either an absolute path or a repo-relative name.
    local input="$1"
    local dir=""

    if [ -d "$input" ]; then
        dir="$input"
    elif [ -d "${REPO_ROOT}/${input}" ]; then
        dir="${REPO_ROOT}/${input}"
    fi

    if [ -n "$dir" ] && [ -f "$dir/docker/compose.yaml" ]; then
        realpath "$dir"
        return 0
    fi

    return 1
}

get_module_default_value() {
    # Read the module's quoted default from template_ws; fallback is handled by caller.
    local module_name="$1"
    local line

    line="$(grep -E "^[[:space:]]*#?[[:space:]]*${module_name}:[[:space:]]*\"" "$TEMPLATE_COMPOSE_FILE" | head -n 1 || true)"
    if [ -z "$line" ]; then
        return 1
    fi

    echo "$line" | sed -E 's/^[^"]*"([^"]*)".*$/\1/'
}

if [ $# -ge 1 ]; then
    MODULE_NAME="$1"
else
    read -rp "Module name to enable (e.g. CARTOGRAPHER): " MODULE_NAME
fi

MODULE_NAME="${MODULE_NAME^^}"
MODULE_NAME="${MODULE_NAME//-/_}"

if [ -z "$MODULE_NAME" ]; then
    echo "Error: Module name is required."
    exit 1
fi

if WORKSPACE_DIR="$(find_current_workspace)"; then
    :
else
    read -rp "Workspace path or name (e.g. template_ws): " WORKSPACE_INPUT
    if ! WORKSPACE_DIR="$(resolve_workspace_dir "$WORKSPACE_INPUT")"; then
        echo "Error: Cannot find a workspace with docker/compose.yaml from: $WORKSPACE_INPUT"
        exit 1
    fi
fi

COMPOSE_FILE="${WORKSPACE_DIR}/docker/compose.yaml"

if ! grep -Eq "^[[:space:]]*#?[[:space:]]*${MODULE_NAME}:[[:space:]]*\"" "$COMPOSE_FILE"; then
    echo "Error: Module ${MODULE_NAME} was not found in ${COMPOSE_FILE}"
    exit 1
fi

MODULE_VALUE="$(get_module_default_value "$MODULE_NAME" || true)"
if [ -z "$MODULE_VALUE" ]; then
    MODULE_VALUE="YES"
fi

sed -Ei \
    "s|^([[:space:]]*)#?[[:space:]]*(${MODULE_NAME}):[[:space:]]*\"[^\"]*\"[[:space:]]*$|\1\2: \"${MODULE_VALUE}\"|" \
    "$COMPOSE_FILE"

echo "Enabled ${MODULE_NAME}=${MODULE_VALUE} in ${COMPOSE_FILE}"
