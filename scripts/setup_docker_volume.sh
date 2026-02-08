#!/bin/bash -e

# Parse command line arguments
RECREATE_VOLUMES=false
while [[ $# -gt 0 ]]; do
	case $1 in
		--recreate-volumes)
			RECREATE_VOLUMES=true
			shift
			;;
		*)
			echo "Unknown option: $1"
			echo "Usage: $0 [--recreate-volumes]"
			echo "  --recreate-volumes  Recreate existing cache volumes"
			exit 1
			;;
	esac
done

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

# Stop containers that may refer to the volumes.
cd "${SCRIPT_DIR}/../template_ws/docker"
docker compose down --remove-orphans

volumes=(ros2-gazebo-cache ros2-isaac-sim-cache ros2-isaac-ros-assets)

# Detect existing volumes.
existing_volumes=()
for v in "${volumes[@]}"; do
	if docker volume inspect "$v" >/dev/null 2>&1; then
		existing_volumes+=("$v")
	fi
done

if [ "${#existing_volumes[@]}" -gt 0 ] && [ "$RECREATE_VOLUMES" = false ]; then
	if [ -t 0 ]; then
		echo "Existing cache volumes detected:"
		for v in "${existing_volumes[@]}"; do
			echo "  - $v"
		done
		echo "Recreate these volumes to invalidate caches (useful after updating/testing Gazebo, Isaac Sim, or Isaac ROS)? [y/N]"
		read -r answer
		case "$answer" in
			[yY]|[yY][eE][sS])
				RECREATE_VOLUMES=true
				;;
		esac
	else
		echo "Keeping existing cache volumes."
		echo "Run with --recreate-volumes to invalidate caches for Gazebo/Isaac Sim/Isaac ROS updates."
	fi
fi

if [ "$RECREATE_VOLUMES" = true ]; then
	for v in "${existing_volumes[@]}"; do
		docker volume rm "$v" >/dev/null
	done
fi

# Ensure docker volumes exist.
for v in "${volumes[@]}"; do
	if ! docker volume inspect "$v" >/dev/null 2>&1; then
		docker volume create "$v" >/dev/null
		echo "Created volume: $v"
	else
		echo "Skipped existing volume: $v"
	fi
done

echo "Set up docker volume done."
