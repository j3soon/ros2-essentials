---
name: vlp-workspace-testing
description: Test and generate proof for vlp_ws. Use when validating VLP workspace build, Docker image CLI health, runtime checks, or Gazebo/RViz simulated lidar screenshot proof.
---

# VLP Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace vlp_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace vlp_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace vlp_ws --level cli
```

## GUI Proof

Runs the documented simulated lidar launch and captures the host X11 screen.
At the time this skill was added, the proof runner preserves a failure if
`spawn_entity.py` cannot reach `/spawn_entity`.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace vlp_ws \
  --summary-json tests/workspace_smoke/artifacts/vlp_ws-doc-demo.json
```
