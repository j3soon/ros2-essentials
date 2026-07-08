#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ARTIFACT_DIR="$REPO_ROOT/tests/workspace_smoke/artifacts"
IMAGE="${ISAAC_LAB_IMAGE:-j3soon/ros2-template-ws}"
CONTAINER_NAME="${ISAAC_LAB_CONTAINER_NAME:-isaac-lab-deformable-proof}"
DISPLAY_VALUE="${DISPLAY:-:0}"
XAUTHORITY_VALUE="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
LOG_PATH="${ISAAC_LAB_LOG_PATH:-$ARTIFACT_DIR/isaac-lab-deformable-kit-gui.log}"
MODE="${1:-run}"

mkdir -p "$ARTIFACT_DIR" "$(dirname "$LOG_PATH")"
docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

docker_args=(
  --name "$CONTAINER_NAME"
  --gpus all
  --privileged
  --network host
  -e "DISPLAY=$DISPLAY_VALUE"
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v /dev:/dev
  --entrypoint bash
)

if [ -r "$XAUTHORITY_VALUE" ]; then
  docker_args+=(
    -e XAUTHORITY=/tmp/docker.xauth
    -v "$XAUTHORITY_VALUE:/tmp/docker.xauth:ro"
  )
fi

container_command=(
  --noprofile
  --norc
  -c
  "cd /home/user/IsaacLab && ./isaaclab.sh -p scripts/tutorials/01_assets/run_deformable_object.py --viz kit"
)

if [ "$MODE" = "--detach" ]; then
  docker run -d "${docker_args[@]}" "$IMAGE" "${container_command[@]}"
  echo "Container: $CONTAINER_NAME"
  echo "Log: docker logs -f $CONTAINER_NAME | tee $LOG_PATH"
  echo "Check screenshot: python3 tests/workspace_smoke/proof_capture.py screenshot --display \"$DISPLAY_VALUE\" --x11-size 1280x720 --output tests/workspace_smoke/artifacts/isaac-lab-deformable-kit-visible-check.png"
  echo "Record after the check screenshot shows rendered orange deformable objects:"
  echo "  python3 tests/workspace_smoke/proof_capture.py record --display \"$DISPLAY_VALUE\" --x11-size 1280x720 --seconds 10 --framerate 15 --output tests/workspace_smoke/artifacts/isaac-lab-deformable-kit-visible.mp4"
else
  docker run --rm "${docker_args[@]}" "$IMAGE" "${container_command[@]}" | tee "$LOG_PATH"
fi
