#!/bin/bash -e

# Parse command line arguments
RECREATE_VOLUMES=false
REMOVE_CONTAINERS=false
while [[ $# -gt 0 ]]; do
	case $1 in
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
			echo "  --recreate-volumes  Recreate existing cache volumes"
			echo "  --remove-containers Remove containers using the cache volumes"
			exit 1
			;;
	esac
done

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

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
		if docker volume rm "$v" >/dev/null 2>&1; then
			echo "Removed volume: $v"
			continue
		fi

		# Find containers still attached to this volume.
		mapfile -t attached_containers < <(docker ps -aq --filter volume="$v")

		if [ "${#attached_containers[@]}" -eq 0 ]; then
			echo "Error: Failed to remove volume '$v'."
			exit 1
		fi

		echo "Volume '$v' is in use by containers:"
		docker ps -a --filter volume="$v" --format "  - {{.ID}} ({{.Names}})"

		if [ "$REMOVE_CONTAINERS" = true ]; then
			for cid in "${attached_containers[@]}"; do
				docker rm -f "$cid" >/dev/null
			done
			docker volume rm "$v" >/dev/null
			echo "Removed volume: $v"
		elif [ -t 0 ]; then
			echo "Remove these containers and retry deleting '$v'? [y/N]"
			read -r answer
			case "$answer" in
				[yY]|[yY][eE][sS])
					for cid in "${attached_containers[@]}"; do
						docker rm -f "$cid" >/dev/null
					done
					docker volume rm "$v" >/dev/null
					echo "Removed volume: $v"
					;;
				*)
					echo "Skipped removing containers. Keeping volume: $v"
					;;
			esac
		else
			echo "Error: Cannot prompt in non-interactive mode."
			echo "Remove containers manually, rerun interactively, or use --remove-containers."
			exit 1
		fi
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
