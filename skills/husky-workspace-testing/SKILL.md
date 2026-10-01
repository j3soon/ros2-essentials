---
name: husky-workspace-testing
description: Test and generate proof for husky_ws. Use when validating the Husky workspace build, Docker image CLI health, runtime checks, or RViz model screenshot proof.
---

# Husky Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace husky_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace husky_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace husky_ws --level cli
```

## GUI Proof

Runs the documented Husky RViz model demo and captures the host X11 screen.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace husky_ws \
  --summary-json tests/workspace_smoke/artifacts/husky_ws-doc-demo.json
```
