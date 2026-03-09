#!/bin/bash -e

# Parse command line arguments
RECREATE_LINKS=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --recreate-links)
            RECREATE_LINKS=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--recreate-links]"
            echo "  --recreate-links  Replace regular files with docker module hard links"
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
        elif [ "$RECREATE_LINKS" = true ]; then
            rm "$file" 2>/dev/null
        else
            if [ -t 0 ]; then
                echo "Found regular files in docker modules (example: $file)."
                echo "Force-remove and recreate module links? [Y/n]"
                echo "Tip: choose 'y' directly if you are not adding/updating docker modules."
                read -r answer
                case "$answer" in
                    [nN]|[nN][oO])
                        echo "Skipped replacement. Remove files manually or rerun with --recreate-links."
                        exit 1
                        ;;
                    *)
                        RECREATE_LINKS=true
                        rm "$file" 2>/dev/null
                        ;;
                esac
            else
                echo "Error: Found regular file instead of hard link: $file."
                echo "Run with --recreate-links to replace these files non-interactively."
                exit 1
            fi
        fi
    done
    for file in docker_modules/*; do
        if [ -f "$file" ]; then
            ln "$file" "${ws}/docker/modules/"
        fi
    done
done

echo "Set up docker modules link done."
