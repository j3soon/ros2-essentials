---
name: aloha-workspace-testing
description: Test and generate proof for aloha_ws. Use when validating the ALOHA workspace build, Docker image CLI health, runtime CLI checks, or RViz robot model screenshot proof.
---

# ALOHA Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace aloha_ws --level build
```

## CLI

Prefer the no-Compose image check unless runtime behavior matters:

```bash
python3 tests/workspace_smoke/run.py --workspace aloha_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace aloha_ws --level cli
```

## GUI Proof

Runs the documented VX300s RViz model demo and captures the host X11 screen.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace aloha_ws \
  --summary-json tests/workspace_smoke/artifacts/aloha_ws-doc-demo.json
```
