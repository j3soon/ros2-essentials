#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ARTIFACT_DIR="$REPO_ROOT/tests/workspace_smoke/artifacts"
WORKSPACE="${ISAAC_LAB_WORKSPACE:-template_ws}"
SERVICE="${ISAAC_LAB_SERVICE:-${WORKSPACE//_/-}}"
COMPOSE_DIR="${ISAAC_LAB_COMPOSE_DIR:-$REPO_ROOT/$WORKSPACE/docker}"
DISPLAY_VALUE="${DISPLAY:-:0}"
LOG_PATH="${ISAAC_LAB_LOG_PATH:-$ARTIFACT_DIR/isaac-lab-deformable-kit-gui.log}"
PID_PATH="${ISAAC_LAB_PID_PATH:-$ARTIFACT_DIR/isaac-lab-deformable-kit-gui.pid}"
MODE="${1:-run}"

LAB_COMMAND="cd /home/user/IsaacLab && ./isaaclab.sh -p scripts/tutorials/01_assets/run_deformable_object.py --viz kit"

mkdir -p "$ARTIFACT_DIR" "$(dirname "$LOG_PATH")"

if [ ! -f "$COMPOSE_DIR/compose.yaml" ]; then
  echo "Compose file not found: $COMPOSE_DIR/compose.yaml" >&2
  exit 1
fi

cd "$COMPOSE_DIR"
docker compose up -d

printf -v exec_command "%q " docker compose exec "$SERVICE" bash -lc "$LAB_COMMAND"

if [ "$MODE" = "--detach" ]; then
  script -qefc "$exec_command" /dev/null >"$LOG_PATH" 2>&1 &
  echo "$!" >"$PID_PATH"
  echo "Workspace: $WORKSPACE"
  echo "Service: $SERVICE"
  echo "Log: tail -f $LOG_PATH"
  echo "PID: $PID_PATH"
  echo "Wait for: Registered backend 'kit', [INFO]: Setup complete, and Root position (in world)"
  echo "Check screenshot: python3 tests/workspace_smoke/proof_capture.py screenshot --display \"$DISPLAY_VALUE\" --x11-size 1280x720 --output tests/workspace_smoke/artifacts/isaac-lab-deformable-kit-visible-check.png"
  echo "Record after the check screenshot shows rendered orange deformable objects:"
  echo "  python3 tests/workspace_smoke/proof_capture.py record --display \"$DISPLAY_VALUE\" --x11-size 1280x720 --seconds 10 --framerate 15 --output tests/workspace_smoke/artifacts/isaac-lab-deformable-kit-visible.mp4"
  echo "Stop: kill \$(cat $PID_PATH); cd $COMPOSE_DIR && docker compose down --remove-orphans"
else
  script -qefc "$exec_command" /dev/null | tee "$LOG_PATH"
fi
