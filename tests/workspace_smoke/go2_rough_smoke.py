#!/usr/bin/env python3
"""Train Go2 briefly, then record pretrained rough-terrain playback on X11."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path

from proof_capture import capture_x11_screenshot, record_x11_video


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPORT_DIR = REPO_ROOT / "tests" / "workspace_smoke" / "artifacts" / "go2-rough"
TASK = "Isaac-Velocity-Rough-UnitreeGo2"
PLAY_MARKER = "[INFO] Policy playback is running"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        default="go2_ws",
        help="Workspace with Isaac Lab installed. Defaults to go2_ws.",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Build the workspace image before testing. Default: reuse the local image.",
    )
    parser.add_argument("--display", default=os.environ.get("DISPLAY") or ":0")
    parser.add_argument("--x11-size", default="1280x720")
    parser.add_argument("--record-seconds", type=int, default=15)
    parser.add_argument("--startup-timeout", type=int, default=480)
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT_DIR)
    parser.add_argument("--summary-json", type=Path)
    return parser.parse_args()


def run_to_log(
    command: list[str], *, cwd: Path, env: dict[str, str], log_path: Path, timeout: int
) -> int:
    print(f"$ {' '.join(command)}\nlog: {log_path}", flush=True)
    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            return subprocess.run(
                command,
                cwd=cwd,
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                timeout=timeout,
            ).returncode
        except subprocess.TimeoutExpired:
            log_file.write(f"Timed out after {timeout} seconds.\n")
            return 124


def wait_for_marker(
    log_path: Path, process: subprocess.Popen, marker: str, timeout: int
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if log_path.is_file() and marker in log_path.read_text(
            encoding="utf-8", errors="replace"
        ):
            return True
        if process.poll() is not None:
            return False
        time.sleep(2)
    return False


def main() -> int:
    args = parse_args()
    if args.record_seconds < 1 or args.startup_timeout < 1:
        raise ValueError("--record-seconds and --startup-timeout must be positive.")

    docker_dir = REPO_ROOT / args.workspace / "docker"
    if not (docker_dir / "compose.yaml").is_file():
        raise ValueError(f"Unknown workspace: {args.workspace}")
    service = args.workspace.replace("_", "-")
    image = f"j3soon/ros2-{service}"
    env = os.environ.copy()
    env["DISPLAY"] = args.display
    env.setdefault("USER_UID", str(os.getuid()))
    run_dir = (
        args.report_dir / args.workspace / datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.summary_json or run_dir / "summary.json"
    artifacts: dict[str, str] = {}
    summary: dict[str, object] = {
        "workspace": args.workspace,
        "status": "failed",
        "artifacts": artifacts,
    }

    def compose(*parts: str) -> list[str]:
        return ["docker", "compose", "-f", "compose.yaml", *parts]

    def artifact(name: str, filename: str) -> Path:
        path = run_dir / filename
        artifacts[name] = (
            str(path.resolve().relative_to(REPO_ROOT))
            if path.resolve().is_relative_to(REPO_ROOT)
            else str(path.resolve())
        )
        return path

    play_process: subprocess.Popen | None = None
    pid_file = f"/tmp/go2-rough-smoke-{os.getpid()}.pid"
    result_code = 0

    def play_group_alive() -> bool:
        command = f"test -f {pid_file} && kill -0 -- -$(cat {pid_file}) 2>/dev/null"
        try:
            return (
                subprocess.run(
                    compose("exec", "-T", service, "bash", "-c", command),
                    cwd=docker_dir,
                    env=env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=10,
                ).returncode
                == 0
            )
        except subprocess.TimeoutExpired:
            return True

    try:
        if (
            not args.build
            and subprocess.run(
                ["docker", "image", "inspect", image],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ).returncode
            != 0
        ):
            raise RuntimeError(
                f"Local image {image} is missing. Build it first or pass --build."
            )
        up_args = (
            "up",
            "-d",
            "--build" if args.build else "--no-build",
            "--pull",
            "never",
        )
        if (
            run_to_log(
                compose(*up_args),
                cwd=docker_dir,
                env=env,
                log_path=artifact("compose_log", "compose-up.log"),
                timeout=7200,
            )
            != 0
        ):
            raise RuntimeError("docker compose up failed. See compose-up.log.")
        if (
            run_to_log(
                compose(
                    "exec",
                    "-T",
                    service,
                    "nvidia-smi",
                    "--query-gpu=name",
                    "--format=csv,noheader",
                ),
                cwd=docker_dir,
                env=env,
                log_path=artifact("gpu_log", "gpu.log"),
                timeout=30,
            )
            != 0
        ):
            raise RuntimeError("The workspace container cannot access the GPU.")

        train_shell = (
            "cd /home/user/IsaacLab && "
            f"timeout --signal=INT --kill-after=30s 900s ./isaaclab.sh train --rl_library rsl_rl --task {TASK} "
            "--num_envs 64 --max_iterations 2"
        )
        train_log = artifact("training_log", "train.log")
        if (
            run_to_log(
                compose("exec", "-T", service, "bash", "-c", train_shell),
                cwd=docker_dir,
                env=env,
                log_path=train_log,
                timeout=960,
            )
            != 0
        ):
            raise RuntimeError("Go2 training failed. See train.log.")
        training_output = train_log.read_text(encoding="utf-8", errors="replace")
        run_name = re.search(
            r"Exact experiment name requested from command line: (\S+)", training_output
        )
        if (
            not run_name
            or "Learning iteration 1/2" not in training_output
            or "Training time:" not in training_output
        ):
            raise RuntimeError(
                "The training log does not confirm two completed iterations."
            )
        checkpoint = artifact("checkpoint", "model_1.pt")
        container_id = subprocess.check_output(
            compose("ps", "-q", service), cwd=docker_dir, env=env, text=True
        ).strip()
        if not container_id:
            raise RuntimeError("The workspace container stopped after training.")
        checkpoint_source = f"{container_id}:/home/user/IsaacLab/logs/rsl_rl/unitree_go2_rough/{run_name.group(1)}/model_1.pt"
        if (
            subprocess.run(
                ["docker", "cp", checkpoint_source, str(checkpoint)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            ).returncode
            != 0
        ):
            raise RuntimeError("Training finished but model_1.pt was not saved.")

        # Isaac Lab 3.0.0-EA accepts these viewer overrides for a camera that follows env 0.
        # The equivalent supported setting for custom tasks is KitVisualizerCfg.
        play_command = (
            f"./isaaclab.sh play --rl_library rsl_rl --task {TASK} --num_envs 4 --checkpoint pretrained --viz kit "
            "env.viewer.origin_type=asset_root env.viewer.asset_name=robot "
            "'env.viewer.eye=[3,3,2]' 'env.viewer.lookat=[0,0,0]'"
        )
        play_shell = (
            "set -e\n"
            "cd /home/user/IsaacLab\n"
            f"setsid {play_command} &\n"
            "play_pid=$!\n"
            f"printf '%s\\n' \"$play_pid\" > {pid_file}\n"
            'wait "$play_pid"\n'
        )
        play_log = artifact("playback_log", "play.log")
        print(
            f"$ docker compose exec -T {service} bash -c <Go2 playback>\nlog: {play_log}",
            flush=True,
        )
        with play_log.open("w", encoding="utf-8") as log_file:
            play_process = subprocess.Popen(
                compose("exec", "-T", service, "bash", "-c", play_shell),
                cwd=docker_dir,
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
            )
            if not wait_for_marker(
                play_log, play_process, PLAY_MARKER, args.startup_timeout
            ):
                raise RuntimeError(
                    "Pretrained Go2 playback did not reach policy readiness. See play.log."
                )
            time.sleep(5)
            if shutil.which("xdotool"):
                windows = subprocess.run(
                    ["xdotool", "search", "--name", "Isaac Lab"],
                    env=env,
                    text=True,
                    capture_output=True,
                )
                if windows.returncode == 0 and windows.stdout.strip():
                    subprocess.run(
                        ["xdotool", "windowactivate", windows.stdout.splitlines()[-1]],
                        env=env,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=10,
                    )
            screenshot = artifact("screenshot", "go2-rough.png")
            if (
                capture_x11_screenshot(
                    screenshot, display=args.display, x11_size=args.x11_size
                )
                != 0
                or screenshot.stat().st_size < 10_000
            ):
                raise RuntimeError("Go2 viewport screenshot failed.")
            recording = artifact("recording", "go2-rough.mp4")
            if (
                record_x11_video(
                    recording,
                    display=args.display,
                    x11_size=args.x11_size,
                    seconds=args.record_seconds,
                    framerate=15,
                )
                != 0
                or recording.stat().st_size < 100_000
            ):
                raise RuntimeError("Go2 viewport recording failed.")
            if play_process.poll() is not None:
                raise RuntimeError(
                    "Go2 playback exited during recording. See play.log."
                )
        summary["status"] = "passed"
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        summary["reason"] = str(error)
        print(f"error: {error}", flush=True)
        result_code = 1
    finally:
        if play_process is not None:
            for signal, grace_seconds in (("INT", 10), ("TERM", 10), ("KILL", 5)):
                if not play_group_alive():
                    break
                try:
                    subprocess.run(
                        compose(
                            "exec",
                            "-T",
                            service,
                            "bash",
                            "-c",
                            f"if test -f {pid_file}; then kill -{signal} -- -$(cat {pid_file}) 2>/dev/null || true; fi",
                        ),
                        cwd=docker_dir,
                        env=env,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=10,
                    )
                    play_process.wait(timeout=grace_seconds)
                except subprocess.TimeoutExpired:
                    pass
            if play_group_alive():
                summary["status"] = "failed"
                summary["reason"] = (
                    "Go2 playback process group did not stop after recording."
                )
                result_code = 1
            try:
                subprocess.run(
                    compose("exec", "-T", service, "bash", "-c", f"rm -f {pid_file}"),
                    cwd=docker_dir,
                    env=env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=10,
                )
            except subprocess.TimeoutExpired:
                pass
            if play_process.poll() is None:
                play_process.terminate()
                try:
                    play_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    play_process.kill()
                    play_process.wait(timeout=5)
                summary["status"] = "failed"
                summary["reason"] = "Go2 playback process did not stop after recording."
                result_code = 1
        summary["generated_at"] = datetime.now().isoformat(timespec="seconds")
        summary["results"] = [
            {
                "workspace": args.workspace,
                "check": "go2-rough",
                "status": summary["status"],
                "log_path": artifacts.get("playback_log")
                or artifacts.get("training_log"),
                "artifact_path": artifacts.get("recording"),
                "artifact_paths": [
                    path
                    for name, path in artifacts.items()
                    if name in {"checkpoint", "screenshot", "recording"}
                ],
                "reason": summary.get("reason"),
            }
        ]
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"Summary: {summary_path}", flush=True)
    return result_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}")
        raise SystemExit(2)
