---
name: kobuki-workspace-testing
description: Test and generate proof for kobuki_ws. Use when validating the Kobuki workspace build, Docker image CLI health, runtime checks, or RViz model screenshot proof.
---

# Kobuki Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace kobuki_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace kobuki_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace kobuki_ws --level cli
```

## GUI Proof

Runs the documented Kobuki RViz model demo and captures the host X11 screen.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace kobuki_ws \
  --summary-json tests/workspace_smoke/artifacts/kobuki_ws-doc-demo.json
```
