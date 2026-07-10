#!/bin/bash -e

# Parse command line arguments
RECREATE_VOLUMES=false
REMOVE_CONTAINERS=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --recreate-links)
            echo "Warning: --recreate-links is deprecated and ignored."
            echo "Docker modules are provided through Compose additional_contexts."
            shift
            ;;
        --recreate-volumes)
            RECREATE_VOLUMES=true
            shift
            ;;
        --remove-containers)
            REMOVE_CONTAINERS=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--recreate-volumes] [--remove-containers]"
            echo "  --recreate-volumes  Recreate Gazebo/Isaac cache volumes"
            echo "  --remove-containers Remove containers that block volume recreation"
            exit 1
            ;;
    esac
done

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

cd "$SCRIPT_DIR"

./pull_latest_docker_image.sh ubuntu:22.04
./setup_env_files.sh

if [ "$RECREATE_VOLUMES" = true ]; then
    if [ "$REMOVE_CONTAINERS" = true ]; then
        ./setup_docker_volume.sh --recreate-volumes --remove-containers
    else
        ./setup_docker_volume.sh --recreate-volumes
    fi
else
    if [ "$REMOVE_CONTAINERS" = true ]; then
        ./setup_docker_volume.sh --remove-containers
    else
        ./setup_docker_volume.sh
    fi
fi

./setup_isaac_link.sh

echo "Post Install Done."
