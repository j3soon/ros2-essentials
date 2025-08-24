#!/bin/bash -e

# Parse command line arguments
FORCE_REMOVE=false
while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--force)
            FORCE_REMOVE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [-f|--force]"
            echo "  -f, --force    Force removal of files even if they're regular files"
            exit 1
            ;;
    esac
done

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

cd "$SCRIPT_DIR/.."

workspaces=( *_ws* )

# Loop through each workspace and link docker modules
for ws in "${workspaces[@]}"
do
    mkdir -p "${ws}/docker/modules"
    for file in "${ws}/docker/modules"/*; do
        # Check if the file is a hard link
        # Ref: https://unix.stackexchange.com/a/167616
        if [ ! -f "$file" ]; then
            continue
        elif [ "$(stat -c %h -- "$file")" -gt 1 ]; then
            rm "$file" 2>/dev/null
        elif [ "$FORCE_REMOVE" = true ]; then
            rm "$file" 2>/dev/null
        else
            echo "Error: Found regular file instead of symlink: $file. Consider removing it manually if not modified."
            exit 1
        fi
    done
    for file in docker_modules/*; do
        if [ -f "$file" ]; then
            ln "$file" "${ws}/docker/modules/"
        fi
    done
done

echo "Set up docker modules link done."
