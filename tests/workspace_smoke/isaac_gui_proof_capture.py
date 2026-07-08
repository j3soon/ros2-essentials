#!/usr/bin/env python3
"""Capture Isaac Sim GUI proof with proper wait logic.

This script coordinates Isaac Sim startup, stage loading, and GUI proof capture.
It waits for ISAAC_GUI_SCENE_READY before capturing screenshots or recordings.

Usage:
    python3 tests/workspace_smoke/isaac_gui_proof_capture.py \\
        --workspace h1_ws \\
        --stage /home/ros2-essentials/h1_ws/isaacsim/assets/h1_og.usda \\
        --expected-prim /World/h1 \\
        --screenshot \\
        --recording --seconds 10

Or for screenshot-only proof with workspace defaults:
    python3 tests/workspace_smoke/isaac_gui_proof_capture.py \\
        --workspace h1_ws \\
        --screenshot
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

CONTAINER_REPO_ROOT = "/home/ros2-essentials"

DEFAULT_STAGE_RELATIVE_PATHS = {
    "h1_ws": "h1_ws/isaacsim/assets/h1_og.usda",
    "go2_ws": "go2_ws/src/isaacsim/assets/go2_og.usda",
    "stretch3_ws": "stretch3_ws/isaacsim/assets/stretch3_og_wasd.usda",
}

DEFAULT_EXPECTED_PRIMS = {
    "h1_ws": "/World/h1",
    "go2_ws": "/World/go2",
    "stretch3_ws": "/World/stretch3",
}


def compose_service(workspace: str) -> str:
    return workspace.replace("_", "-")


def compose_dir(workspace: str) -> Path:
    return REPO_ROOT / workspace / "docker"


def container_repo_path(relative_path: str) -> str:
    return f"{CONTAINER_REPO_ROOT}/{relative_path}"


def host_path_for_stage(stage: str) -> Path:
    container_prefix = f"{CONTAINER_REPO_ROOT}/"
    if stage.startswith(container_prefix):
        return REPO_ROOT / stage.removeprefix(container_prefix)
    path = Path(stage)
    if path.is_absolute():
        return path
    return REPO_ROOT / stage


def container_path_for_stage(stage: str) -> str:
    container_prefix = f"{CONTAINER_REPO_ROOT}/"
    if stage.startswith(container_prefix):
        return stage
    path = Path(stage)
    if path.is_absolute():
        try:
            return container_repo_path(str(path.resolve().relative_to(REPO_ROOT)))
        except ValueError:
            return stage
    return container_repo_path(stage)


def wait_for_isaac_ready(log_file: Path, timeout: int = 180) -> bool:
    """Wait for Isaac Sim to report stage ready."""
    start = time.time()
    ready_markers = [
        "ISAAC_GUI_SCENE_READY",
        "ISAAC_GUI_EXPECTED_PRIM_OK",
        "ISAAC_GUI_TIMELINE_PLAYING",
    ]

    while time.time() - start < timeout:
        if log_file.exists():
            content = log_file.read_text()
            if any(marker in content for marker in ready_markers):
                print(f"Isaac Sim ready after {time.time() - start:.1f}s", flush=True)
                return True
        time.sleep(1)

    print(f"Isaac Sim not ready after {timeout}s", flush=True)
    return False


def capture_screenshot(display: str, output: Path, x11_size: str = "1280x720") -> bool:
    """Capture screenshot from X11 display."""
    cmd = [
        sys.executable, str(REPO_ROOT / "tests/workspace_smoke/proof_capture.py"),
        "screenshot",
        "--display", display,
        "--x11-size", x11_size,
        "--output", str(output),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"Screenshot saved to {output}", flush=True)
        return True
    print(f"Screenshot failed: {result.stderr}", flush=True)
    return False


def capture_recording(display: str, output: Path, seconds: int = 10, framerate: int = 15) -> bool:
    """Capture recording from X11 display."""
    cmd = [
        sys.executable, str(REPO_ROOT / "tests/workspace_smoke/proof_capture.py"),
        "record",
        "--display", display,
        "--x11-size", "1280x720",
        "--seconds", str(seconds),
        "--framerate", str(framerate),
        "--output", str(output),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        print(f"Recording saved to {output}", flush=True)
        return True
    print(f"Recording failed: {result.stderr}", flush=True)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_argument_group("workspace selection")
    selection.add_argument("--workspace", required=True, help="Workspace name (e.g., h1_ws)")
    selection.add_argument("--stage", help="USD stage path (defaults to workspace-specific stage)")
    selection.add_argument("--expected-prim", help="Expected prim path (e.g., /World/h1)")

    capture = parser.add_argument_group("capture options")
    capture.add_argument("--screenshot", action="store_true", help="Capture screenshot")
    capture.add_argument("--recording", action="store_true", help="Capture recording")
    capture.add_argument("--seconds", type=int, default=10, help="Recording duration")
    capture.add_argument("--framerate", type=int, default=15, help="Recording framerate")
    capture.add_argument("--output-dir", help="Output directory for artifacts")
    capture.add_argument("--display", default=os.environ.get("DISPLAY", ":0"), help="X11 display")

    timing = parser.add_argument_group("timing options")
    timing.add_argument("--timeout", type=int, default=180, help="Timeout for Isaac Sim ready")
    timing.add_argument("--settle-seconds", type=int, default=10, help="Seconds to wait before capture")

    args = parser.parse_args()

    workspace_compose_dir = compose_dir(args.workspace)
    if not (workspace_compose_dir / "compose.yaml").is_file():
        print(f"Compose file not found: {workspace_compose_dir / 'compose.yaml'}", file=sys.stderr)
        return False

    output_dir = Path(args.output_dir) if args.output_dir else REPO_ROOT / "tests/workspace_smoke/artifacts" / args.workspace
    output_dir.mkdir(parents=True, exist_ok=True)

    stage = args.stage
    if not stage:
        if args.workspace not in DEFAULT_STAGE_RELATIVE_PATHS:
            print(f"No default Isaac stage for workspace: {args.workspace}", file=sys.stderr)
            return False
        stage = container_repo_path(DEFAULT_STAGE_RELATIVE_PATHS[args.workspace])
    stage_host_path = host_path_for_stage(stage)
    if not stage_host_path.is_file():
        print(f"Stage file not found: {stage}", file=sys.stderr)
        return False
    stage = container_path_for_stage(stage)

    expected_prim = args.expected_prim or DEFAULT_EXPECTED_PRIMS.get(args.workspace, "")

    isaac_cmd = [
        "docker", "compose",
        "exec", "-e", f"DISPLAY={args.display}",
        "-e", "XAUTHORITY=/home/user/.Xauthority",
        "-e", f"ISAAC_GUI_STAGE_PATH={stage}",
    ]
    if expected_prim:
        isaac_cmd.extend(["-e", f"ISAAC_GUI_EXPECTED_PRIM={expected_prim}"])
    isaac_cmd.extend([
        "-e", "ISAAC_GUI_PLAY=true",
        "-e", "ISAAC_GUI_SETTLE_FRAMES=180",
        compose_service(args.workspace),
        "bash", "-lc",
        "/home/user/isaacsim/isaac-sim.sh --exec "
        "/home/ros2-essentials/tests/workspace_smoke/isaac_gui_open_stage.py",
    ])

    log_file = output_dir / "isaac-gui.log"
    log_file.write_text("", encoding="utf-8")
    print(f"Starting Isaac Sim with stage: {stage}", flush=True)
    print(f"Log file: {log_file}", flush=True)

    up_cmd = ["docker", "compose", "up", "-d"]
    with log_file.open("a", encoding="utf-8") as log:
        log.write(f"$ {' '.join(up_cmd)}\n")
        up_result = subprocess.run(
            up_cmd,
            cwd=workspace_compose_dir,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
    if up_result.returncode != 0:
        print("docker compose up failed", file=sys.stderr)
        return False

    proc = subprocess.Popen(
        isaac_cmd,
        cwd=workspace_compose_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    def stream_logs():
        assert proc.stdout is not None
        with log_file.open("a", encoding="utf-8") as log:
            for line in proc.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()

    log_thread = threading.Thread(target=stream_logs, daemon=True)
    log_thread.start()

    results = []
    try:
        if not wait_for_isaac_ready(log_file, args.timeout):
            print("Isaac Sim failed to become ready, taking screenshot anyway...", flush=True)

        print(f"Waiting {args.settle_seconds}s for scene to settle...", flush=True)
        time.sleep(args.settle_seconds)

        if args.screenshot:
            output_file = output_dir / f"{args.workspace}-isaac-proof.png"
            results.append(("screenshot", capture_screenshot(args.display, output_file)))

        if args.recording:
            output_file = output_dir / f"{args.workspace}-isaac-recording.mp4"
            results.append(("recording", capture_recording(args.display, output_file, args.seconds, args.framerate)))
    finally:
        print("Stopping Isaac Sim...", flush=True)
        proc.terminate()
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=30)
        subprocess.run(
            ["docker", "compose", "down", "--remove-orphans"],
            cwd=workspace_compose_dir,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    try:
        log_thread.join(timeout=5)
    except RuntimeError:
        pass

    print("\n=== Proof Capture Summary ===", flush=True)
    for name, success in results:
        status = "PASS" if success else "FAIL"
        print(f"  {status} {name}", flush=True)

    return all(success for _, success in results)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
