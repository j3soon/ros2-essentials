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

## Full Proof with Recording

For screenshot + recording proof with proper wait logic:

```bash
python3 tests/workspace_smoke/isaac_gui_proof_capture.py \
  --workspace h1_ws \
  --screenshot --recording --seconds 10
```

**Key timing notes:**
- Isaac Sim 6.0.1 takes 30-60s to launch
- Stage loading takes additional 10-20s
- Wait for `ISAAC_GUI_SCENE_READY` or `ISAAC_GUI_TIMELINE_PLAYING` in logs
- Add 10s settle time before capture to avoid black frames
- Recording requires X11 display (`DISPLAY=:0`) and proper window visibility
