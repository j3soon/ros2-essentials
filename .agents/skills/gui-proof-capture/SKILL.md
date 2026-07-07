---
name: gui-proof-capture
description: Reusable host-side GUI proof capture for robotics workspace tests. Use when taking screenshots or short recordings of X11-rendered Gazebo, RViz, Isaac Sim, or other Docker-launched GUI applications for proof artifacts.
---

# GUI Proof Capture

Use `tests/workspace_smoke/proof_capture.py` from the repository root. The GUI
application may run inside Docker, but the capture runs on the host X11 display.

Screenshot:

```bash
python3 tests/workspace_smoke/proof_capture.py screenshot \
  --display "${DISPLAY:-:0}" \
  --x11-size 1280x720 \
  --output tests/workspace_smoke/artifacts/<name>.png
```

Short recording:

```bash
python3 tests/workspace_smoke/proof_capture.py record \
  --display "${DISPLAY:-:0}" \
  --x11-size 1280x720 \
  --seconds 10 \
  --framerate 15 \
  --output tests/workspace_smoke/artifacts/<name>.mp4
```

The helper wakes the display with `xset` and captures with `ffmpeg -f x11grab`.
For automated workspace demos, prefer
`tests/workspace_smoke/doc_demo_smoke.py`; it already calls this helper and
writes screenshot artifacts beside logs.
