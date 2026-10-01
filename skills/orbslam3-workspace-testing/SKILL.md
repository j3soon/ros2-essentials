---
name: orbslam3-workspace-testing
description: Test and generate proof for orbslam3_ws. Use when validating ORB-SLAM3 workspace build, Docker image CLI health, runtime checks, or documenting that full demo proof requires the external EuRoC dataset.
---

# ORB-SLAM3 Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace orbslam3_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace orbslam3_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace orbslam3_ws --level cli
```

## GUI Proof

The documented demo requires an external EuRoC dataset and bag playback, so the
automated doc-demo runner records this as skipped.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace orbslam3_ws \
  --summary-json tests/workspace_smoke/artifacts/orbslam3_ws-doc-demo.json
```
