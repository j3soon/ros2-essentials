#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

cd "$SCRIPT_DIR/.."

workspaces=( *_ws* )

# Loop through each workspace and link docker modules
for ws in "${workspaces[@]}"
do
    mkdir -p "${ws}/docker/modules"
    rm -f "${ws}/docker/modules"/* 2>/dev/null || true
    for file in docker_modules/*; do
        if [ -f "$file" ]; then
            ln "$file" "${ws}/docker/modules/"
        fi
    done
done

echo "Set up docker modules link done."
