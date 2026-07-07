---
name: go2-workspace-testing
description: Test and generate proof for go2_ws. Use when validating the Go2 workspace build, Docker image CLI health, runtime checks, or Isaac Sim stage screenshot proof.
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

Runs the documented Isaac Sim stage and validates `/World/go2` before taking a
host X11 screenshot.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace go2_ws \
  --summary-json tests/workspace_smoke/artifacts/go2_ws-doc-demo.json
```
