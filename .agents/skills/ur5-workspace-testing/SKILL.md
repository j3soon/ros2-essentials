---
name: ur5-workspace-testing
description: Test and generate proof for ur5_ws. Use when validating UR5 workspace build, Docker image CLI health, runtime checks, or documenting URSim-dependent proof limits.
---

# UR5 Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace ur5_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace ur5_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace ur5_ws --level cli
```

## GUI Proof

The documented simulator depends on a separate URSim container and robot
calibration parameters, so the doc-demo runner records this as skipped.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace ur5_ws \
  --summary-json tests/workspace_smoke/artifacts/ur5_ws-doc-demo.json
```
