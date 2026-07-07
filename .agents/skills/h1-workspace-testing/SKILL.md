---
name: h1-workspace-testing
description: Test and generate proof for h1_ws. Use when validating the H1 workspace build, Docker image CLI health, runtime checks, or Isaac Sim stage screenshot proof.
---

# H1 Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace h1_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace h1_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace h1_ws --level cli
```

## GUI Proof

Runs the documented Isaac Sim stage and validates `/World/h1` before taking a
host X11 screenshot.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace h1_ws \
  --summary-json tests/workspace_smoke/artifacts/h1_ws-doc-demo.json
```
