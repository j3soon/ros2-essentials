---
name: stretch3-workspace-testing
description: Test and generate proof for stretch3_ws. Use when validating Stretch 3 workspace build, Docker image CLI health, runtime checks, or Isaac Sim stage screenshot proof.
---

# Stretch 3 Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace stretch3_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace stretch3_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace stretch3_ws --level cli
```

## GUI Proof

Runs the documented Isaac Sim stage and validates `/World/stretch3` before
taking a host X11 screenshot.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace stretch3_ws \
  --summary-json tests/workspace_smoke/artifacts/stretch3_ws-doc-demo.json
```
