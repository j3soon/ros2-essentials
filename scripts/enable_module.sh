#!/bin/bash -e

# Usage: ./scripts/enable_module.sh <MODULE_NAME>
#        ./scripts/enable_module.sh --list-modules

# Dev Note:
#   Please note that this script is mostly vibe coded.
#   Briefly checked by eye but not line-by-line.

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

list_available_workspaces() {
    # Print repo workspaces that contain docker/compose.yaml.
    find "$REPO_ROOT" -mindepth 1 -maxdepth 1 -type d -name '*_ws' \
        | while read -r dir; do
            if [ -f "$dir/docker/compose.yaml" ]; then
                basename "$dir"
            fi
        done \
        | sort -u
}

resolve_workspace_input() {
    # Resolve workspace from path, exact name, or unique prefix.
    local input="$1"
    local dir
    local normalized
    local names=()
    local matches=()
    local name

    if dir="$(resolve_workspace_dir "$input" 2>/dev/null)"; then
        echo "$dir"
        return 0
    fi

    normalized="${input,,}"
    normalized="${normalized//-/_}"

    mapfile -t names < <(list_available_workspaces)
    if [ "${#names[@]}" -eq 0 ]; then
        return 1
    fi

    for name in "${names[@]}"; do
        if [ "${name,,}" = "$normalized" ]; then
            resolve_workspace_dir "$name"
            return 0
        fi
    done

    for name in "${names[@]}"; do
        if [[ "${name,,}" == "$normalized"* ]]; then
            matches+=("$name")
        fi
    done

    if [ "${#matches[@]}" -eq 1 ]; then
        resolve_workspace_dir "${matches[0]}"
        return 0
    fi

    return 1
}

get_module_default_value() {
    # Read the module's quoted default from template_ws; fallback is handled by caller.
    local module_name="$1"
    local line

    line="$(
        awk '
            /^[[:space:]]*args:[[:space:]]*$/ { in_args=1; next }
            in_args && /^[[:space:]]*cache_from:[[:space:]]*$/ { exit }
            in_args { print }
        ' "$TEMPLATE_COMPOSE_FILE" \
        | grep -E "^[[:space:]]*#?[[:space:]]*${module_name}:[[:space:]]*\"" \
        | head -n 1 || true
    )"
    if [ -z "$line" ]; then
        return 1
    fi

    echo "$line" | sed -E 's/^[^"]*"([^"]*)".*$/\1/'
}

list_available_modules() {
    # Print module keys from template_ws build.args for selection and matching.
    awk '
        /^[[:space:]]*args:[[:space:]]*$/ { in_args=1; next }
        in_args && /^[[:space:]]*cache_from:[[:space:]]*$/ { exit }
        in_args { print }
    ' "$TEMPLATE_COMPOSE_FILE" \
    | sed -nE 's/^[[:space:]]*#?[[:space:]]*([A-Z0-9_]+):[[:space:]]*".*"/\1/p' \
    | grep -Ev '^USER_UID$' \
    | sort -u
}

resolve_module_name() {
    # Resolve a module from exact name or a unique prefix (case-insensitive).
    local input="$1"
    local normalized
    local modules=()
    local matches=()
    local module

    normalized="${input^^}"
    normalized="${normalized//-/_}"

    mapfile -t modules < <(list_available_modules)
    if [ "${#modules[@]}" -eq 0 ]; then
        return 1
    fi

    for module in "${modules[@]}"; do
        if [ "$module" = "$normalized" ]; then
            echo "$module"
            return 0
        fi
    done

    for module in "${modules[@]}"; do
        if [[ "$module" == "$normalized"* ]]; then
            matches+=("$module")
        fi
    done

    if [ "${#matches[@]}" -eq 1 ]; then
        echo "${matches[0]}"
        return 0
    fi

    return 1
}

choose_module_name() {
    # Show a select menu and accept number, full name, or unique prefix.
    local modules=()
    local input
    local resolved
    local old_ps3="${PS3-#? }"
    mapfile -t modules < <(list_available_modules)

    if [ "${#modules[@]}" -eq 0 ]; then
        return 1
    fi

    echo "Select a module to enable." >&2
    echo "Input can be a number, module name, or unique prefix (case-insensitive)." >&2
    PS3="Module: "
    select module in "${modules[@]}"; do
        if [ -n "$module" ]; then
            PS3="$old_ps3"
            echo "$module"
            return 0
        fi
        input="$REPLY"
        if [ -z "$input" ]; then
            PS3="$old_ps3"
            return 1
        fi
        if resolved="$(resolve_module_name "$input")"; then
            PS3="$old_ps3"
            echo "$resolved"
            return 0
        fi
        echo "Invalid or ambiguous module. Type more characters and try again." >&2
    done
}

choose_workspace_dir() {
    # Show a select menu and accept number, name/path, or unique prefix.
    local workspaces=()
    local input
    local resolved
    local old_ps3="${PS3-#? }"
    mapfile -t workspaces < <(list_available_workspaces)

    if [ "${#workspaces[@]}" -eq 0 ]; then
        return 1
    fi

    echo "Select a workspace." >&2
    echo "Input can be a number, workspace name/path, or unique prefix (case-insensitive)." >&2
    PS3="Workspace: "
    select ws in "${workspaces[@]}"; do
        if [ -n "$ws" ]; then
            PS3="$old_ps3"
            resolve_workspace_dir "$ws"
            return 0
        fi
        input="$REPLY"
        if [ -z "$input" ]; then
            PS3="$old_ps3"
            return 1
        fi
        if resolved="$(resolve_workspace_input "$input")"; then
            PS3="$old_ps3"
            echo "$resolved"
            return 0
        fi
        echo "Invalid or ambiguous workspace. Type more characters and try again." >&2
    done
}

if [ "${1:-}" = "--list-modules" ]; then
    list_available_modules
    exit 0
fi

if [ $# -ge 1 ]; then
    MODULE_NAME="$1"
else
    if ! MODULE_NAME="$(choose_module_name)"; then
        read -rp "Module name to enable (e.g. CARTOGRAPHER): " MODULE_NAME
    fi
fi

if MODULE_NAME_RESOLVED="$(resolve_module_name "$MODULE_NAME")"; then
    MODULE_NAME="$MODULE_NAME_RESOLVED"
else
    MODULE_NAME="${MODULE_NAME^^}"
    MODULE_NAME="${MODULE_NAME//-/_}"
fi

if [ -z "$MODULE_NAME" ]; then
    echo "Error: Module name is required."
    exit 1
fi

if WORKSPACE_DIR="$(find_current_workspace)"; then
    :
else
    if ! WORKSPACE_DIR="$(choose_workspace_dir)"; then
        read -rp "Workspace path, name, or unique prefix (e.g. template_ws): " WORKSPACE_INPUT
        if [ -z "$WORKSPACE_INPUT" ]; then
            echo "Error: Workspace input is required."
            exit 1
        fi
    fi
    if [ -n "${WORKSPACE_INPUT:-}" ]; then
        WORKSPACE_DIR="$(resolve_workspace_input "$WORKSPACE_INPUT" || true)"
    fi
    if [ -z "${WORKSPACE_DIR:-}" ]; then
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
