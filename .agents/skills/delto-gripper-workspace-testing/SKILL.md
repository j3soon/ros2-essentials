---
name: delto-gripper-workspace-testing
description: Test and generate proof for delto_gripper_ws. Use when validating the Delto gripper workspace build, Docker image CLI health, runtime checks, or Isaac Sim stage screenshot proof.
---

# Delto Gripper Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace delto_gripper_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace delto_gripper_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace delto_gripper_ws --level cli
```

## GUI Proof

Runs the documented Isaac Sim stage and validates `/dg5f_right` before taking a
host X11 screenshot.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace delto_gripper_ws \
  --summary-json tests/workspace_smoke/artifacts/delto_gripper_ws-doc-demo.json
```
