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
			echo "  -f, --force    Force removal of existing volumes"
			exit 1
			;;
	esac
done

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
# Create .env directory if it doesn't exist.
mkdir -p "${SCRIPT_DIR}/../.env"
# Create .env/.claude directory if it doesn't exist.
mkdir -p "${SCRIPT_DIR}/../.env/.claude"
# Create .env/.claude.json file if it doesn't exist.
touch "${SCRIPT_DIR}/../.env/.claude.json"
# Create .env/.codex directory if it doesn't exist.
mkdir -p "${SCRIPT_DIR}/../.env/.codex"

# Run docker compose down to remove containers referring to the volumes.
cd "${SCRIPT_DIR}/../template_ws/docker"
docker compose down --volumes --remove-orphans

volumes=(ros2-gazebo-cache ros2-isaac-sim-cache ros2-isaac-ros-assets)

# Check for existing volumes
for v in "${volumes[@]}"; do
	if docker volume inspect "$v" >/dev/null 2>&1; then
		if [ "$FORCE_REMOVE" = true ]; then
			docker volume rm "$v" >/dev/null
		else
			echo "Error: Docker volume '$v' already exists."
			echo "Run with -f to remove existing volumes and continue."
			exit 1
		fi
	fi
done

# Create docker volumes
for v in "${volumes[@]}"; do
	docker volume create "$v" >/dev/null
done

echo "Set up docker volume done."
