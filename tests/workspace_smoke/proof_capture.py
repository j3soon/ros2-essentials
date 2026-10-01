#!/usr/bin/env python3
"""Shared host-side screenshot and recording helpers for workspace proofs."""

from __future__ import annotations

import argparse
import os
import subprocess
import time
from pathlib import Path


def wake_x11_display(display: str) -> None:
    subprocess.run(
        ["xset", "-display", display, "dpms", "force", "on"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    subprocess.run(
        ["xset", "-display", display, "s", "reset"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(1)


def require_x11_window_visible(window_id: str, *, display: str) -> None:
    """Reject hidden windows or a desktop whose foreground window changed."""
    env = {**os.environ, "DISPLAY": display}
    visible = subprocess.run(
        ["xdotool", "search", "--onlyvisible", "--name", ".*"],
        env=env,
        text=True,
        capture_output=True,
        timeout=10,
    )
    active = subprocess.run(
        ["xdotool", "getactivewindow"],
        env=env,
        text=True,
        capture_output=True,
        timeout=10,
    )
    if (
        visible.returncode != 0
        or window_id not in visible.stdout.splitlines()
        or active.returncode != 0
        or active.stdout.strip() != window_id
    ):
        raise RuntimeError(
            f"Proof window {window_id} is not visible in the foreground."
        )


def capture_x11_screenshot(
    path: Path, *, display: str, x11_size: str, window_id: str | None = None
) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    wake_x11_display(display)
    if window_id is not None:
        require_x11_window_visible(window_id, display=display)
    command = [
        "ffmpeg",
        "-y",
        "-f",
        "x11grab",
        "-video_size",
        x11_size,
        *(["-window_id", window_id] if window_id is not None else []),
        "-i",
        display,
        "-frames:v",
        "1",
        str(path),
    ]
    return subprocess.run(
        command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    ).returncode


def record_x11_video(
    path: Path,
    *,
    display: str,
    x11_size: str,
    seconds: int,
    framerate: int,
    window_id: str | None = None,
) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    wake_x11_display(display)
    if window_id is not None:
        require_x11_window_visible(window_id, display=display)
    command = [
        "ffmpeg",
        "-y",
        "-f",
        "x11grab",
        "-video_size",
        x11_size,
        "-framerate",
        str(framerate),
        *(["-window_id", window_id] if window_id is not None else []),
        "-i",
        display,
        "-t",
        str(seconds),
        "-codec:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-pix_fmt",
        "yuv420p",
        str(path),
    ]
    if window_id is None:
        return subprocess.run(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ).returncode
    with subprocess.Popen(
        command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    ) as process:
        try:
            while True:
                require_x11_window_visible(window_id, display=display)
                try:
                    return process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    pass
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_common(subparser: argparse.ArgumentParser) -> None:
        subparser.add_argument("--output", type=Path, required=True)
        subparser.add_argument("--display", default=":0")
        subparser.add_argument("--x11-size", default="1280x720")

    screenshot = subparsers.add_parser("screenshot")
    add_common(screenshot)

    record = subparsers.add_parser("record")
    add_common(record)
    record.add_argument("--seconds", type=int, default=10)
    record.add_argument("--framerate", type=int, default=15)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "screenshot":
        return capture_x11_screenshot(
            args.output, display=args.display, x11_size=args.x11_size
        )
    if args.command == "record":
        return record_x11_video(
            args.output,
            display=args.display,
            x11_size=args.x11_size,
            seconds=args.seconds,
            framerate=args.framerate,
        )
    raise ValueError(f"unsupported command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
