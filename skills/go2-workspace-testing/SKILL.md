---
name: go2-workspace-testing
description: Test and generate proof for go2_ws. Use when validating the Go2 workspace build, Docker image CLI health, runtime checks, or Isaac Sim Go2 joint-command motion proof.
---

# Go2 Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace go2_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace go2_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace go2_ws --level cli
```

## GUI Proof

Runs the documented Custom Isaac Sim Environment demo. The runner opens
`go2_og.usda` with the Isaac timeline paused, validates `/World/go2`, starts
host X11 recording, triggers Isaac timeline Play, verifies ROS traffic with
`ros2 topic echo --once /clock` and `ros2 topic echo --once /joint_states`,
waits for the scene to stabilize, publishes the documented `/joint_command`,
then captures a final host X11 screenshot. The MP4 must include the start of
simulation and pre-command frames.
The doc-demo runner starts Compose with `docker compose up -d --build --pull
never`, so it must build the local workspace image and must not pull the
published DockerHub image.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace go2_ws \
  --summary-json tests/workspace_smoke/artifacts/go2_ws-doc-demo.json
```

Report back with the summary JSON path, log path, screenshot path, and MP4
recording path.
