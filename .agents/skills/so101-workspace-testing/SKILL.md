---
name: so101-workspace-testing
description: Test and generate proof for so101_ws. Use when validating SO-101 workspace build, Docker image CLI health, runtime checks, Isaac visual startup, or documenting hardware-dependent proof limits.
---

# SO-101 Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace so101_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace so101_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace so101_ws --level cli
```

## GUI Proof

The documented flows require real SO-101 hardware, camera, or interactive
teleoperation, so the doc-demo runner records this as skipped.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace so101_ws \
  --summary-json tests/workspace_smoke/artifacts/so101_ws-doc-demo.json
```
