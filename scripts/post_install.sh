#!/bin/bash -e

# Parse command line arguments
FORCE_FLAG=""
while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--force)
            FORCE_FLAG="-f"
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

cd "$SCRIPT_DIR"

./setup_docker_modules_link.sh $FORCE_FLAG
./setup_docker_volume.sh
./setup_isaac_link.sh

echo "Post Install Done."
