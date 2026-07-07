---
name: turtlebot3-workspace-testing
description: Test and generate proof for turtlebot3_ws. Use when validating TurtleBot3 workspace build, Docker image CLI health, runtime checks, or RViz fake-node screenshot proof.
---

# TurtleBot3 Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace turtlebot3_ws --level cli
```

## GUI Proof

Runs the documented fake-node RViz demo with `TURTLEBOT3_MODEL=burger` and
captures the host X11 screen.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace turtlebot3_ws \
  --summary-json tests/workspace_smoke/artifacts/turtlebot3_ws-doc-demo.json
```
