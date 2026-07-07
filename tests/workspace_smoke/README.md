# Workspace Smoke Tests

This directory contains opt-in smoke tests for ROS2 workspaces. These tests are
separate from `tests/test_all.sh` because they may build Docker images, start
containers, or require local graphics/GPU setup.

## Test Levels

- `config`: run `docker compose config` for each selected workspace.
- `build`: run `config`, then `docker compose build`.
- `image-cli`: run basic ROS/colcon commands in the already-built image with
  `docker run`, without starting the GPU-requesting Compose service.
- `runtime`: run `config`, `docker compose up -d`, `docker compose ps`,
  collect logs, and `docker compose down`.
- `cli`: run `runtime` plus basic commands inside the primary workspace
  service.
- `isaac-visual`: run the already-built image with GPU access, start Isaac Sim
  headless, render a deterministic cube/ground/camera scene, and write a PNG
  proof artifact next to the command log. The runner first validates host
  Docker GPU startup with `docker run --gpus all ... nvidia-smi`; if that
  preflight fails, workspace visual checks are skipped because they cannot
  start either. It then starts one minimal Isaac Sim `SimulationApp`; if RTX
  renderer startup fails there, workspace visual checks are skipped because the
  common renderer path cannot produce screenshots.
- `gui`: reserved for manual/local GUI validation placeholders.

`doc_demo_smoke.py` is the local proof runner for representative commands from
workspace docs. It starts GUI/Isaac demos, captures screenshots under
`tests/workspace_smoke/artifacts/doc-demo/`, and writes a reusable summary JSON.
For Isaac doc demos it launches the normal Isaac Sim GUI, validates the
expected robot prim, and captures the full desktop/window rather than a cropped
render.

`proof_capture.py` is the shared host-side X11 capture helper used by GUI proof
runs. It can also be called directly for reusable screenshots or short
recordings after any container renders to the host display.

Use `--no-gpu` when the host does not have a GPU mounted. In that mode, the
runner allows `config`, `build`, `image-cli`, and `gui`, but rejects checks that
start or inspect running Compose services, including `isaac-visual`.
Use `--disable-gpu-reservation` when Docker GPU startup is blocked by host
NVIDIA runtime state and you still want runtime/CLI proof without editing
workspace compose files. The generated override is written under the artifact
directory and does not apply to `isaac-visual`.

## Examples

List workspaces:

```bash
python3 tests/workspace_smoke/run.py --list-workspaces
```

Run the lightweight CI-safe check for one workspace:

```bash
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level config --no-gpu
```

Build one workspace image:

```bash
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level build
```

Run basic ROS/colcon checks inside an already-built image without Compose:

```bash
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level image-cli --no-gpu
```

Run a GPU-backed Isaac Sim screenshot check in an already-built image:

```bash
python3 tests/workspace_smoke/run.py --workspace so101_ws --level isaac-visual
```

Run only the visual path after a host GPU service fix:

```bash
python3 tests/workspace_smoke/run.py \
  --all \
  --check isaac-visual \
  --summary-json tests/workspace_smoke/artifacts/isaac-visual-after-host-fix.json
```

Start a workspace container and run basic CLI checks:

```bash
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level cli
```

Start a workspace container without requesting the host GPU:

```bash
python3 tests/workspace_smoke/run.py \
  --workspace turtlebot3_ws \
  --level cli \
  --disable-gpu-reservation
```

Run checks for workspaces affected by a branch:

```bash
python3 tests/workspace_smoke/run.py --changed-from main --changed-to HEAD --level config --no-gpu
```

Run checks from a changed-file list:

```bash
git diff --name-only main...HEAD | \
  python3 tests/workspace_smoke/run.py --changed-files-from - --level config
```

Run selected checks directly:

```bash
python3 tests/workspace_smoke/run.py \
  --workspace turtlebot3_ws \
  --check config --check up --check cli --check logs --check down
```

When `down` is part of the selected checks, the runner also attempts it after a
failed earlier runtime check so broad proof runs do not leave compose stacks up.

Run an all-workspace proof pass without stopping at the first failure:

```bash
python3 tests/workspace_smoke/run.py \
  --all \
  --check config --check image-cli \
  --continue-on-failure \
  --summary-json tests/workspace_smoke/artifacts/all-workspaces-summary.json
```

Run Isaac visual proof checks for all already-built workspace images:

```bash
python3 tests/workspace_smoke/run.py \
  --all \
  --check isaac-visual \
  --continue-on-failure \
  --summary-json tests/workspace_smoke/artifacts/isaac-visual-summary.json
```

Run documented workspace demos and save screenshot proof:

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py \
  --all \
  --continue-on-failure \
  --summary-json tests/workspace_smoke/artifacts/doc-demo-summary.json
```

Use `--image-override <workspace>=<image>` when validating a locally patched
image before retagging it as the default workspace image.

Capture the current host X11 display directly:

```bash
python3 tests/workspace_smoke/proof_capture.py screenshot \
  --display "${DISPLAY:-:0}" \
  --x11-size 1280x720 \
  --output tests/workspace_smoke/artifacts/manual-proof.png
```

Record a short host X11 proof clip:

```bash
python3 tests/workspace_smoke/proof_capture.py record \
  --display "${DISPLAY:-:0}" \
  --x11-size 1280x720 \
  --seconds 10 \
  --framerate 15 \
  --output tests/workspace_smoke/artifacts/manual-proof.mp4
```

If the host has no mounted GPU, keep the selected checks to `config`, `build`,
or `image-cli`:

```bash
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level build --no-gpu
```

## Affected Workspace Rules

Changed paths under `<name>_ws/` affect only that workspace.

Changes under shared infrastructure paths affect all workspaces:

- `docker_modules/`
- `scripts/post_install.sh`
- `scripts/setup_docker_modules_link.sh`
- `scripts/setup_env_files.sh`
- `scripts/setup_isaac_link.sh`
- `.agents/skills/`
- `tests/workspace_smoke/`
- `.github/workflows/workspace-smoke.yaml`

Docs-only changes do not select Docker smoke tests unless they are accompanied
by workspace or shared infrastructure changes.

## Artifacts

Command logs are written under `tests/workspace_smoke/artifacts/` by default.
The `isaac-visual` check also writes a PNG with the same basename as its log.
`--summary-json` writes a reusable status manifest with log and PNG artifact
paths. This directory is ignored by Git.

Generate a Markdown proof report from saved manifests:

```bash
python3 tests/workspace_smoke/report.py \
  --summary config-image-cli=tests/workspace_smoke/artifacts/all-workspaces-config-image-cli.json \
  --summary runtime-cli=tests/workspace_smoke/artifacts/all-workspaces-runtime-cli-disable-gpu-reservation-v2.json \
  --summary isaac-visual=tests/workspace_smoke/artifacts/all-workspaces-isaac-visual-preflight.json \
  --test-log test-all=tests/workspace_smoke/artifacts/test-all-final.log \
  --output tests/workspace_smoke/artifacts/proof-report.md
```
