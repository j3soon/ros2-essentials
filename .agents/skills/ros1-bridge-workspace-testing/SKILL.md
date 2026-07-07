---
name: ros1-bridge-workspace-testing
description: Test and generate proof for ros1_bridge_ws. Use when validating ROS 1 Bridge build, Docker image CLI health, runtime checks, or documenting bridge demo proof limitations.
---

# ROS 1 Bridge Workspace Testing

## Build

This workspace is memory-sensitive during compilation. Use the repository
Dockerfile as-is so its bounded build parallelism is preserved.

```bash
python3 tests/workspace_smoke/run.py --workspace ros1_bridge_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace ros1_bridge_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace ros1_bridge_ws --level cli
```

## GUI Proof

There is no screenshot surface for the bridge demo. The automated doc-demo
runner records this as skipped until a multi-service text proof is added.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace ros1_bridge_ws \
  --summary-json tests/workspace_smoke/artifacts/ros1_bridge_ws-doc-demo.json
```
