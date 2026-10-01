---
name: gazebo-world-workspace-testing
description: Test and generate proof for gazebo_world_ws. Use when validating Gazebo world workspace build, Docker image CLI health, runtime checks, or Gazebo screenshot proof.
---

# Gazebo World Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace gazebo_world_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace gazebo_world_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace gazebo_world_ws --level cli
```

## GUI Proof

Runs the documented TurtleBot3 Gazebo world demo and captures the host X11
screen.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace gazebo_world_ws \
  --summary-json tests/workspace_smoke/artifacts/gazebo_world_ws-doc-demo.json
```
