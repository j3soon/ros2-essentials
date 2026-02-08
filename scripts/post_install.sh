#!/bin/bash -e

# Parse command line arguments
RECREATE_LINKS=false
RECREATE_VOLUMES=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --recreate-links)
            RECREATE_LINKS=true
            shift
            ;;
        --recreate-volumes)
            RECREATE_VOLUMES=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--recreate-links] [--recreate-volumes]"
            echo "  --recreate-links    Replace non-hard-link docker module files"
            echo "  --recreate-volumes  Recreate Gazebo/Isaac cache volumes"
            exit 1
            ;;
    esac
done

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

cd "$SCRIPT_DIR"

./setup_env_files.sh

if [ "$RECREATE_LINKS" = true ]; then
    ./setup_docker_modules_link.sh --recreate-links
else
    ./setup_docker_modules_link.sh
fi

if [ "$RECREATE_VOLUMES" = true ]; then
    ./setup_docker_volume.sh --recreate-volumes
else
    ./setup_docker_volume.sh
fi

./setup_isaac_link.sh

echo "Post Install Done."
