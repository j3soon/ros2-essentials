# Isaac Sim GUI Proof Capture Guide

## Quick Start

### Screenshot Only
```bash
python3 tests/workspace_smoke/isaac_gui_proof_capture.py \
  --workspace h1_ws \
  --screenshot
```

### Screenshot + Recording
```bash
python3 tests/workspace_smoke/isaac_gui_proof_capture.py \
  --workspace h1_ws \
  --screenshot --recording --seconds 10
```

## Manual Workflow

For full control over the proof capture process:

```bash
# 1. Start the workspace
cd h1_ws/docker && docker compose up -d

# 2. Launch Isaac Sim with the stage in background
docker compose exec -e DISPLAY=:0 -e XAUTHORITY=/home/user/.Xauthority \
  h1-ws bash -lc 'export ISAAC_GUI_STAGE_PATH=/home/ros2-essentials/h1_ws/isaacsim/assets/h1_og.usda; \
  export ISAAC_GUI_PLAY=true; \
  /home/user/isaacsim/isaac-sim.sh --exec /home/ros2-essentials/tests/workspace_smoke/isaac_gui_open_stage.py' &

# 3. Wait for Isaac Sim to be ready (check for these markers in logs):
#    - ISAAC_GUI_STAGE_OPENED
#    - ISAAC_GUI_EXPECTED_PRIM_OK
#    - ISAAC_GUI_TIMELINE_PLAYING
#    - ISAAC_GUI_SCENE_READY

# 4. Wait additional 10-20s for scene to settle (avoid black frames)
sleep 15

# 5. Capture screenshot
python3 tests/workspace_smoke/proof_capture.py screenshot \
  --display :0 \
  --x11-size 1280x720 \
  --output tests/workspace_smoke/artifacts/h1_ws/isaac-proof.png

# 6. Capture recording
python3 tests/workspace_smoke/proof_capture.py record \
  --display :0 \
  --x11-size 1280x720 \
  --seconds 10 \
  --framerate 15 \
  --output tests/workspace_smoke/artifacts/h1_ws/isaac-recording.mp4

# 7. Cleanup
docker compose down --remove-orphans
```

## Common Issues

### Empty/Black Recording
- Isaac Sim window may not be visible on the host display
- Wait longer for scene to settle (15-30s after stage opens)
- Check that `DISPLAY=:0` is accessible from the terminal running capture
- Verify Isaac Sim GUI window is actually rendering (use `xdotool` or manual check)

### Isaac Sim Not Ready
- Increase `--timeout` (default 180s)
- Check logs for errors: `docker compose logs h1-ws | tail -100`
- Some extensions may fail to load (e.g., `isaacsim.robot_motion.pink`) - this is expected

### X11 Display Issues
- Ensure X11 forwarding is configured: `xhost +local:docker`
- Verify DISPLAY is set: `echo $DISPLAY`
- Test display access: `xdpyinfo`

## Artifacts Location

All proof artifacts are saved to:
```
tests/workspace_smoke/artifacts/<workspace>/
  - <workspace>-isaac-proof.png
  - <workspace>-isaac-recording.mp4
  - isaac-gui.log
```

## Isaac Sim Version Verification

To verify the Isaac Sim version in the proof:
```bash
# Check in logs
grep "Isaac Sim Full Version" tests/workspace_smoke/artifacts/<workspace>/isaac-gui.log

# Expected output: [XX.Xs] Isaac Sim Full Version: 6.0.1-rc.7
```
