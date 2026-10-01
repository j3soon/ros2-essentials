---
name: go2-workspace-testing
description: Test and generate proof for go2_ws. Use for workspace build and runtime checks, Isaac Sim joint-command motion, or Isaac Lab rough-terrain training and playback.
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

Runs the documented Custom Isaac Sim Environment demo. The runner opens
`go2_og.usda` with the Isaac timeline paused, validates `/World/go2`, starts
host X11 recording, triggers Isaac timeline Play, verifies ROS traffic with
`ros2 topic echo --once /clock` and `ros2 topic echo --once /joint_states`,
waits for the scene to stabilize, publishes the documented `/joint_command`,
then captures a final host X11 screenshot. The MP4 must include the start of
simulation and pre-command frames.
The doc-demo runner builds locally and refuses registry pulls. After a
successful build, add `--no-build` to reuse the tested image.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace go2_ws \
  --summary-json tests/workspace_smoke/artifacts/go2_ws-doc-demo.json
```

Report back with the summary JSON path, log path, screenshot path, and MP4
recording path.

## Isaac Lab Rough-Terrain Proof

The Go2 Isaac Lab check runs two RSL-RL training iterations, verifies and
copies `model_1.pt`, then loads the published pretrained policy with the Kit
visualizer. It captures a screenshot and a 15-second X11 recording after the
playback readiness marker. The host needs `xdotool`, `xprop`, and `ffmpeg`. The runner
requires one newly visible Isaac Lab window and captures that window. Keep it
in the foreground throughout recording. Missing, ambiguous, hidden, or
covered windows fail the proof. Inspect the screenshot and a video frame to
confirm that Go2 and the terrain rendered.
Use a local image that includes Isaac Lab 3.0.0-EA.
The runner reuses it without rebuilding and leaves the Compose service running
so the logs and checkpoint remain available inside the container.
Its camera-following CLI overrides work in 3.0.0-EA but are deprecated. Use
`KitVisualizerCfg` in custom task code, as shown in `docs/go2-ws/index.md`.

```bash
python3 tests/workspace_smoke/go2_rough_smoke.py --workspace go2_ws
```

If only the fully built template image is available, pass
`--workspace template_ws`. Pass `--build` to build the chosen workspace first.
Report the summary JSON, training log, checkpoint, screenshot, and MP4 paths.
