#!/usr/bin/env python3
"""Run opt-in Docker smoke tests for selected ROS2 workspaces."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPORT_DIR = REPO_ROOT / "tests" / "workspace_smoke" / "artifacts"

CHECKS = (
    "config",
    "build",
    "image-cli",
    "up",
    "ps",
    "cli",
    "logs",
    "down",
    "isaac-visual",
    "isaac-lab-deformable",
    "gui",
)
LEVELS = {
    "config": ("config",),
    "build": ("config", "build"),
    "image-cli": ("image-cli",),
    "runtime": ("config", "up", "ps", "logs", "down"),
    "cli": ("config", "up", "ps", "cli", "logs", "down"),
    "isaac-visual": ("isaac-visual",),
    "isaac-lab-deformable": ("isaac-lab-deformable",),
    "gui": ("gui",),
}
SHARED_PATHS = (
    ".agents/skills/",
    ".github/workflows/workspace-smoke.yaml",
    "docker_modules/",
    "scripts/post_install.sh",
    "scripts/setup_env_files.sh",
    "scripts/setup_isaac_link.sh",
    "tests/workspace_smoke/",
)


@dataclass(frozen=True)
class Workspace:
    name: str
    path: Path
    docker_dir: Path
    service: str
    image: str


@dataclass
class Result:
    workspace: str
    check: str
    status: str
    log_path: Path | None = None
    artifact_path: Path | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run decoupled smoke checks for ROS2 workspace Docker setups."
    )
    selection = parser.add_argument_group("workspace selection")
    selection.add_argument(
        "--workspace",
        action="append",
        default=[],
        help="Workspace name or path, for example turtlebot3_ws. May be repeated.",
    )
    selection.add_argument(
        "--all",
        action="store_true",
        help="Select every *_ws workspace.",
    )
    selection.add_argument(
        "--changed-file",
        action="append",
        default=[],
        help="Changed file path used to infer affected workspaces. May be repeated.",
    )
    selection.add_argument(
        "--changed-files-from",
        help="Read changed file paths from a file, or '-' for stdin.",
    )
    selection.add_argument(
        "--changed-from",
        help="Base git ref used with git diff --name-only to infer changed paths.",
    )
    selection.add_argument(
        "--changed-to",
        default="HEAD",
        help="Head git ref used with --changed-from. Defaults to HEAD.",
    )
    selection.add_argument(
        "--list-workspaces",
        action="store_true",
        help="Print discovered workspaces and exit.",
    )

    checks = parser.add_argument_group("checks")
    checks.add_argument(
        "--level",
        choices=sorted(LEVELS),
        default="config",
        help="Convenience check group. Defaults to config.",
    )
    checks.add_argument(
        "--check",
        action="append",
        choices=CHECKS,
        default=[],
        help="Run an individual check. May be repeated; overrides --level.",
    )
    checks.add_argument(
        "--cli-command",
        default=(
            "set -e; "
            "pwd; "
            "if [ -f install/setup.bash ]; then "
            "source install/setup.bash; "
            "elif [ -f /opt/ros/humble/setup.bash ]; then "
            "source /opt/ros/humble/setup.bash; "
            "elif [ -f /ros2_humble/install/setup.bash ]; then "
            "source /ros2_humble/install/setup.bash; "
            "fi; "
            "command -v ros2; "
            "ros2 pkg list > /tmp/ros2-pkgs.txt; "
            "test -s /tmp/ros2-pkgs.txt; "
            "printf 'ros2 package count: '; "
            "wc -l < /tmp/ros2-pkgs.txt; "
            "if [ -d src ] && command -v colcon >/dev/null 2>&1; then "
            "colcon list; "
            "elif command -v colcon >/dev/null 2>&1; then "
            "colcon --help >/dev/null; "
            "fi"
        ),
        help="Command executed inside the primary service for the cli check.",
    )
    checks.add_argument(
        "--image-cli-command",
        default=(
            "set -e; "
            "if [ -f /opt/ros/humble/setup.bash ]; then "
            "source /opt/ros/humble/setup.bash; "
            "elif [ -f /ros2_humble/install/setup.bash ]; then "
            "source /ros2_humble/install/setup.bash; "
            "else "
            "echo 'No ROS 2 setup.bash found' >&2; exit 1; "
            "fi; "
            "command -v ros2; "
            "python3 --version; "
            "ros2 pkg list > /tmp/ros2-pkgs.txt; "
            "test -s /tmp/ros2-pkgs.txt; "
            "printf 'ros2 package count: '; "
            "wc -l < /tmp/ros2-pkgs.txt; "
            "printf 'sample packages:\\n'; "
            "head -20 /tmp/ros2-pkgs.txt; "
            "if command -v colcon >/dev/null 2>&1; then colcon --help >/dev/null; fi"
        ),
        help=(
            "Command executed with docker run against the built image for the "
            "image-cli check."
        ),
    )
    checks.add_argument(
        "--pull",
        action="store_true",
        help="Pass --pull to docker compose build.",
    )
    checks.add_argument(
        "--no-gpu",
        action="store_true",
        help=(
            "Reject runtime checks that start Compose services with GPU device "
            "requests. Safe checks are config, build, and gui."
        ),
    )
    checks.add_argument(
        "--disable-gpu-reservation",
        action="store_true",
        help=(
            "Use a generated Compose override that removes the primary service "
            "GPU reservation for runtime/cli fallback checks. This does not "
            "apply to Isaac GPU checks."
        ),
    )
    checks.add_argument(
        "--post-install",
        action="store_true",
        help="Run scripts/post_install.sh before workspace checks.",
    )
    checks.add_argument(
        "--log-tail",
        type=int,
        default=200,
        help="Number of lines collected by docker compose logs. Defaults to 200.",
    )
    checks.add_argument(
        "--isaac-visual-timeout",
        type=int,
        default=300,
        help="Seconds allowed for the Isaac visual screenshot check. Defaults to 300.",
    )
    checks.add_argument(
        "--isaac-lab-timeout",
        type=int,
        default=480,
        help="Seconds allowed for Isaac Lab smoke checks. Defaults to 480.",
    )
    checks.add_argument(
        "--gpu-preflight-image",
        default="nvidia/cuda:12.4.1-base-ubuntu22.04",
        help=(
            "Image used to validate Docker GPU startup before Isaac GPU "
            "checks. Defaults to nvidia/cuda:12.4.1-base-ubuntu22.04."
        ),
    )
    checks.add_argument(
        "--skip-gpu-preflight",
        action="store_true",
        help="Skip the Docker GPU startup preflight before Isaac GPU checks.",
    )
    checks.add_argument(
        "--skip-isaac-startup-preflight",
        action="store_true",
        help=(
            "Skip the Isaac Sim SimulationApp startup preflight before "
            "isaac-visual checks."
        ),
    )
    checks.add_argument(
        "--report-dir",
        type=Path,
        default=DEFAULT_REPORT_DIR,
        help="Directory for command logs.",
    )
    checks.add_argument(
        "--dry-run",
        action="store_true",
        help="Print selected workspaces and checks without running commands.",
    )
    checks.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Continue running remaining checks after a workspace check fails.",
    )
    checks.add_argument(
        "--summary-json",
        type=Path,
        help="Write a JSON summary containing statuses, logs, and proof artifacts.",
    )
    return parser.parse_args()


def display_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def discover_workspaces() -> dict[str, Workspace]:
    workspaces: dict[str, Workspace] = {}
    for path in sorted(REPO_ROOT.glob("*_ws")):
        docker_dir = path / "docker"
        compose_file = docker_dir / "compose.yaml"
        if not compose_file.is_file():
            continue
        name = path.name
        service = name.replace("_", "-")
        if name == "ros1_bridge_ws":
            service = "ros1-bridge"
        workspaces[name] = Workspace(
            name=name,
            path=path,
            docker_dir=docker_dir,
            service=service,
            image=f"j3soon/ros2-{name.replace('_', '-')}",
        )
    return workspaces


def normalize_workspace_name(value: str) -> str:
    return Path(value.rstrip("/")).name


def read_changed_files(args: argparse.Namespace) -> list[str]:
    changed = list(args.changed_file)

    if args.changed_files_from:
        if args.changed_files_from == "-":
            changed.extend(line.strip() for line in sys.stdin if line.strip())
        else:
            source = Path(args.changed_files_from)
            changed.extend(line.strip() for line in source.read_text().splitlines() if line.strip())

    if args.changed_from:
        diff_range = f"{args.changed_from}...{args.changed_to}"
        command = ["git", "diff", "--name-only", "--diff-filter=ACMRTUXB", diff_range]
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if completed.returncode == 0:
            changed.extend(line.strip() for line in completed.stdout.splitlines() if line.strip())
        else:
            print(
                "warning: unable to diff changed refs; selecting all workspaces. "
                f"Command failed: {' '.join(command)}",
                file=sys.stderr,
            )
            if completed.stderr.strip():
                print(completed.stderr.strip(), file=sys.stderr)
            changed.append(SHARED_PATHS[0])

    normalized = []
    for path in changed:
        path = path.strip()
        if not path:
            continue
        if path.startswith("./"):
            path = path[2:]
        normalized.append(path)
    return sorted(set(normalized))


def is_shared_path(path: str) -> bool:
    return any(path == shared.rstrip("/") or path.startswith(shared) for shared in SHARED_PATHS)


def affected_workspaces(
    changed_files: Iterable[str], workspaces: dict[str, Workspace]
) -> tuple[set[str], bool]:
    selected: set[str] = set()
    shared_change = False
    for path in changed_files:
        first = path.split("/", 1)[0]
        if first in workspaces:
            selected.add(first)
        elif is_shared_path(path):
            shared_change = True
    if shared_change:
        selected.update(workspaces)
    return selected, shared_change


def select_workspaces(args: argparse.Namespace, workspaces: dict[str, Workspace]) -> list[Workspace]:
    if args.all:
        return list(workspaces.values())

    selected: set[str] = set()
    for value in args.workspace:
        name = normalize_workspace_name(value)
        if name not in workspaces:
            raise ValueError(f"Unknown workspace: {value}")
        selected.add(name)

    changed_files = read_changed_files(args)
    if changed_files:
        affected, shared_change = affected_workspaces(changed_files, workspaces)
        selected.update(affected)
        print("Changed files:")
        for path in changed_files:
            print(f"  {path}")
        if shared_change:
            print("Shared infrastructure changed; selecting all workspaces.")

    if not selected:
        print("No affected workspaces selected.")
        return []

    return [workspaces[name] for name in sorted(selected)]


def selected_checks(args: argparse.Namespace) -> tuple[str, ...]:
    if args.check:
        return tuple(args.check)
    return LEVELS[args.level]


def validate_gpu_mode(args: argparse.Namespace, checks: tuple[str, ...]) -> None:
    if not args.no_gpu:
        return
    gpu_runtime_checks = {
        "up",
        "ps",
        "cli",
        "logs",
        "down",
        "isaac-visual",
        "isaac-lab-deformable",
    }
    blocked = [check for check in checks if check in gpu_runtime_checks]
    if blocked:
        blocked_text = ", ".join(blocked)
        raise ValueError(
            "--no-gpu cannot run checks that start or inspect GPU-requesting "
            f"Compose services: {blocked_text}. Use --level config, --level build, "
            "or --level image-cli."
        )


def command_log_path(report_dir: Path, workspace: Workspace, check: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return report_dir / workspace.name / f"{timestamp}-{check}.log"


def host_log_path(report_dir: Path, check: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return report_dir / "host" / f"{timestamp}-{check}.log"


def gpu_override_path(report_dir: Path, workspace: Workspace) -> Path:
    override_path = report_dir / workspace.name / "disable-gpu-reservation.compose.yaml"
    override_path.parent.mkdir(parents=True, exist_ok=True)
    override_path.write_text(
        "\n".join(
            [
                "services:",
                f"  {workspace.service}:",
                "    deploy:",
                "      resources:",
                "        reservations:",
                "          devices: !reset []",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return override_path


def run_command(
    command: list[str],
    *,
    cwd: Path,
    env: dict[str, str],
    log_path: Path,
    input_text: str | None = None,
) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"$ {' '.join(command)}")
    print(f"log: {log_path.relative_to(REPO_ROOT)}")
    with log_path.open("w", encoding="utf-8") as log_file:
        log_file.write(f"$ {' '.join(command)}\n")
        process = subprocess.Popen(
            command,
            cwd=cwd,
            env=env,
            text=True,
            stdin=subprocess.PIPE if input_text is not None else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
        )
        if input_text is not None:
            assert process.stdin is not None
            process.stdin.write(input_text)
            process.stdin.close()
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="")
            log_file.write(line)
        return process.wait()


def compose_command(
    workspace: Workspace,
    args: list[str],
    override_file: Path | None = None,
) -> list[str]:
    command = ["docker", "compose"]
    if override_file is not None:
        command.extend(["-f", "compose.yaml", "-f", str(override_file)])
    command.extend(args)
    return command


def check_config(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    override_file: Path | None,
) -> int:
    return run_command(compose_command(workspace, ["config"], override_file), cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_build(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    pull: bool,
    override_file: Path | None,
) -> int:
    args = ["build"]
    if pull:
        args.append("--pull")
    return run_command(compose_command(workspace, args, override_file), cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_up(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    override_file: Path | None,
) -> int:
    return run_command(compose_command(workspace, ["up", "-d"], override_file), cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_ps(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    override_file: Path | None,
) -> int:
    return run_command(compose_command(workspace, ["ps", "--all"], override_file), cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_cli(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    cli_command: str,
    override_file: Path | None,
) -> int:
    command = compose_command(workspace, ["exec", "-T", workspace.service, "bash", "-lc", cli_command], override_file)
    return run_command(command, cwd=workspace.docker_dir, env=env, log_path=log_path)


def image_name(workspace: Workspace) -> str:
    return workspace.image


def with_pty(command: str) -> str:
    return f"script -qefc {shlex.quote(command)} /dev/null"


def compose_shell_command(
    workspace: Workspace,
    shell_command: str,
    *,
    tty: bool,
) -> str:
    exec_args = ["docker", "compose", "exec"]
    if not tty:
        exec_args.append("-T")
    exec_args.extend([workspace.service, "bash", "-lc", shell_command])
    command = " ".join(shlex.quote(part) for part in exec_args)
    return with_pty(command) if tty else command


def check_compose_exec(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    shell_command: str,
    *,
    timeout_seconds: int,
    tty: bool,
) -> int:
    command = (
        "set -e; "
        "docker compose up -d; "
        "code=0; "
        f"timeout {timeout_seconds:d}s "
        f"{compose_shell_command(workspace, shell_command, tty=tty)} || code=$?; "
        "docker compose down --remove-orphans; "
        "exit $code"
    )
    return run_command(["bash", "-lc", command], cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_image_cli(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    image_cli_command: str,
) -> int:
    image = image_name(workspace)
    command = (
        f"docker image inspect {shlex.quote(image)} >/dev/null && "
        f"docker run --rm --network host --entrypoint bash {shlex.quote(image)} "
        f"-c {shlex.quote(image_cli_command)}"
    )
    return run_command(["bash", "-lc", command], cwd=REPO_ROOT, env=env, log_path=log_path)


def check_isaac_visual(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    timeout_seconds: int,
) -> int:
    output_path = log_path.with_suffix(".png")
    container_script = "/home/ros2-essentials/tests/workspace_smoke/isaac_visual_smoke.py"
    container_output = f"/home/ros2-essentials/{display_path(output_path)}"
    shell_command = (
        "/home/user/isaacsim/python.sh "
        f"{shlex.quote(container_script)} "
        f"--output {shlex.quote(container_output)} "
        f"--label {shlex.quote(workspace.name)}"
    )
    return check_compose_exec(
        workspace,
        env,
        log_path,
        shell_command,
        timeout_seconds=timeout_seconds,
        tty=False,
    )


def check_isaac_lab_deformable(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    timeout_seconds: int,
) -> int:
    tutorial_command = (
        "./isaaclab.sh -p "
        "scripts/tutorials/01_assets/run_deformable_object.py --viz kit"
    )
    start_command = (
        "set -e; "
        "test -x /home/user/IsaacLab/isaaclab.sh; "
        "command -v script >/dev/null; "
        "cd /home/user/IsaacLab; "
        "log=/tmp/isaac-lab-deformable.log; "
        "typescript=/tmp/isaac-lab-deformable.typescript; "
        "pidfile=/tmp/isaac-lab-deformable.pid; "
        "rm -f \"$log\" \"$typescript\" \"$pidfile\"; "
        f"nohup script -qefc {shlex.quote(tutorial_command)} \"$typescript\" "
        ">\"$log\" 2>&1 </dev/null & "
        "echo $! >\"$pidfile\""
    )
    markers_command = (
        "log=/tmp/isaac-lab-deformable.log; "
        "typescript=/tmp/isaac-lab-deformable.typescript; "
        "grep -q \"Registered backend 'kit'\" \"$log\" \"$typescript\" 2>/dev/null "
        "&& grep -q '\\[INFO\\]: Setup complete' \"$log\" \"$typescript\" 2>/dev/null "
        "&& grep -q 'Root position (in world)' \"$log\" \"$typescript\" 2>/dev/null"
    )
    print_markers_command = (
        "log=/tmp/isaac-lab-deformable.log; "
        "typescript=/tmp/isaac-lab-deformable.typescript; "
        "grep -hm 12 \"Registered backend 'kit'\\|\\[INFO\\]: Setup complete\\|Root position (in world)\" "
        "\"$log\" \"$typescript\" 2>/dev/null"
    )
    process_alive_command = (
        "pidfile=/tmp/isaac-lab-deformable.pid; "
        "test -f \"$pidfile\" && kill -0 \"$(cat \"$pidfile\")\" 2>/dev/null"
    )
    tail_command = (
        "log=/tmp/isaac-lab-deformable.log; "
        "typescript=/tmp/isaac-lab-deformable.typescript; "
        "tail -n 160 \"$log\" \"$typescript\" 2>/dev/null || true"
    )
    exec_prefix = "docker compose exec -T " + shlex.quote(workspace.service) + " bash -lc "
    command = (
        "set -e; "
        "docker compose up -d; "
        "code=124; "
        f"{exec_prefix}{shlex.quote(start_command)}; "
        f"deadline=$((SECONDS + {timeout_seconds:d})); "
        "while [ \"$SECONDS\" -lt \"$deadline\" ]; do "
        f"if {exec_prefix}{shlex.quote(markers_command)} >/dev/null 2>&1; then "
        "echo 'Isaac Lab deformable readiness markers found.'; "
        f"{exec_prefix}{shlex.quote(print_markers_command)}; "
        "code=0; "
        "break; "
        "fi; "
        f"if ! {exec_prefix}{shlex.quote(process_alive_command)} >/dev/null 2>&1; then "
        "echo 'Isaac Lab deformable process exited before readiness markers.' >&2; "
        f"{exec_prefix}{shlex.quote(tail_command)}; "
        "code=1; "
        "break; "
        "fi; "
        "sleep 2; "
        "done; "
        "if [ \"$code\" -eq 124 ]; then "
        "echo 'Timed out waiting for Isaac Lab deformable readiness markers.' >&2; "
        f"{exec_prefix}{shlex.quote(tail_command)}; "
        "fi; "
        "docker compose down --remove-orphans; "
        "exit $code"
    )
    return run_command(["bash", "-lc", command], cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_gpu_preflight(
    env: dict[str, str],
    log_path: Path,
    image: str,
) -> int:
    command = (
        "set -e; "
        "echo 'Host NVIDIA state:'; "
        "nvidia-smi; "
        "echo; "
        "echo 'Docker GPU startup:'; "
        f"docker run --rm --gpus all {shlex.quote(image)} nvidia-smi"
    )
    return run_command(["bash", "-lc", command], cwd=REPO_ROOT, env=env, log_path=log_path)


def check_isaac_startup_preflight(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    timeout_seconds: int,
) -> int:
    shell_command = (
        "/home/user/isaacsim/python.sh -c "
        + shlex.quote(
            "from isaacsim import SimulationApp; "
            "print('ISAAC_STARTUP_BEFORE'); "
            "app=SimulationApp({'headless': True, 'limit_cpu_threads': 16}); "
            "print('ISAAC_STARTUP_AFTER'); "
            "app.close(); "
            "print('ISAAC_STARTUP_CLOSED')"
        )
    )
    return check_compose_exec(
        workspace,
        env,
        log_path,
        shell_command,
        timeout_seconds=timeout_seconds,
        tty=False,
    )


def check_logs(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    log_tail: int,
    override_file: Path | None,
) -> int:
    command = compose_command(workspace, ["logs", "--no-color", "--tail", str(log_tail)], override_file)
    return run_command(command, cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_down(
    workspace: Workspace,
    env: dict[str, str],
    log_path: Path,
    override_file: Path | None,
) -> int:
    command = compose_command(workspace, ["down", "--remove-orphans"], override_file)
    return run_command(command, cwd=workspace.docker_dir, env=env, log_path=log_path)


def check_gui(workspace: Workspace) -> None:
    print(
        f"[SKIP] {workspace.name}: GUI validation is intentionally manual/local-only for now. "
        "Use this level to track future Isaac Sim, Isaac Lab, Gazebo, screenshot, or recording checks."
    )


def supports_isaac_gpu_check(workspace: Workspace) -> bool:
    return workspace.name != "ros1_bridge_ws"


def run_post_install(env: dict[str, str], report_dir: Path) -> int:
    log_path = report_dir / "post-install.log"
    return run_command(
        ["scripts/post_install.sh"],
        cwd=REPO_ROOT,
        env=env,
        log_path=log_path,
        input_text="n\n",
    )


def run_gpu_preflight(args: argparse.Namespace, env: dict[str, str]) -> Result:
    log_path = host_log_path(args.report_dir, "gpu-preflight")
    code = check_gpu_preflight(env, log_path, args.gpu_preflight_image)
    status = "passed" if code == 0 else "failed"
    return Result("host", "gpu-preflight", status, log_path)


def run_isaac_startup_preflight(
    args: argparse.Namespace,
    env: dict[str, str],
    selected: list[Workspace],
) -> Result | None:
    workspace = next((item for item in selected if item.name != "ros1_bridge_ws"), None)
    if workspace is None:
        return None
    log_path = host_log_path(args.report_dir, "isaac-startup-preflight")
    code = check_isaac_startup_preflight(
        workspace,
        env,
        log_path,
        args.isaac_visual_timeout,
    )
    status = "passed" if code == 0 else "failed"
    return Result("host", "isaac-startup-preflight", status, log_path)


def run_check(
    workspace: Workspace,
    check: str,
    args: argparse.Namespace,
    env: dict[str, str],
) -> Result:
    if check == "gui":
        check_gui(workspace)
        return Result(workspace.name, check, "skipped")
    if check in {"isaac-visual", "isaac-lab-deformable"} and not supports_isaac_gpu_check(workspace):
        print(f"[SKIP] {workspace.name}: {check} is not applicable to this workspace image.")
        return Result(workspace.name, check, "skipped")

    log_path = command_log_path(args.report_dir, workspace, check)
    artifact_path = log_path.with_suffix(".png") if check == "isaac-visual" else None
    override_file = (
        gpu_override_path(args.report_dir, workspace)
        if args.disable_gpu_reservation and check != "isaac-visual"
        else None
    )
    if check == "config":
        code = check_config(workspace, env, log_path, override_file)
    elif check == "build":
        code = check_build(workspace, env, log_path, args.pull, override_file)
    elif check == "up":
        code = check_up(workspace, env, log_path, override_file)
    elif check == "ps":
        code = check_ps(workspace, env, log_path, override_file)
    elif check == "cli":
        code = check_cli(workspace, env, log_path, args.cli_command, override_file)
    elif check == "image-cli":
        code = check_image_cli(workspace, env, log_path, args.image_cli_command)
    elif check == "isaac-visual":
        code = check_isaac_visual(workspace, env, log_path, args.isaac_visual_timeout)
    elif check == "isaac-lab-deformable":
        code = check_isaac_lab_deformable(workspace, env, log_path, args.isaac_lab_timeout)
    elif check == "logs":
        code = check_logs(workspace, env, log_path, args.log_tail, override_file)
    elif check == "down":
        code = check_down(workspace, env, log_path, override_file)
    else:
        raise ValueError(f"Unsupported check: {check}")

    status = "passed" if code == 0 else "failed"
    if artifact_path and not artifact_path.is_file():
        artifact_path = None
    return Result(workspace.name, check, status, log_path, artifact_path)


def print_summary(results: list[Result]) -> None:
    print()
    print("Summary:")
    if not results:
        print("  No workspace smoke checks ran.")
        return
    for result in results:
        detail = ""
        if result.log_path:
            detail = f" ({display_path(result.log_path)})"
        if result.artifact_path:
            detail += f" artifact={display_path(result.artifact_path)}"
        print(f"  {result.workspace}: {result.check}: {result.status}{detail}")


def write_summary_json(path: Path, results: list[Result]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "results": [
            {
                "workspace": result.workspace,
                "check": result.check,
                "status": result.status,
                "log_path": (
                    display_path(result.log_path)
                    if result.log_path
                    else None
                ),
                "artifact_path": (
                    display_path(result.artifact_path)
                    if result.artifact_path
                    else None
                ),
            }
            for result in results
        ],
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    workspaces = discover_workspaces()

    if args.list_workspaces:
        for workspace in workspaces.values():
            print(workspace.name)
        return 0

    checks = selected_checks(args)
    validate_gpu_mode(args, checks)
    selected = select_workspaces(args, workspaces)

    print("Selected workspaces:")
    for workspace in selected:
        print(f"  {workspace.name}")
    print("Selected checks:")
    for check in checks:
        print(f"  {check}")

    if args.dry_run or not selected:
        return 0

    env = os.environ.copy()
    env.setdefault("USER_UID", str(os.getuid()))

    if args.post_install:
        code = run_post_install(env, args.report_dir)
        if code != 0:
            print("post_install failed; stopping before workspace checks.")
            return code

    results: list[Result] = []
    isaac_gpu_checks = {"isaac-visual", "isaac-lab-deformable"}
    needs_isaac_gpu = any(check in isaac_gpu_checks for check in checks)
    if needs_isaac_gpu and not args.skip_gpu_preflight:
        print()
        print("== host ==")
        result = run_gpu_preflight(args, env)
        results.append(result)
        if result.status == "failed":
            print(
                "GPU preflight failed; skipping Isaac GPU workspace checks. "
                "Verify `docker run --rm --gpus all ... nvidia-smi` before "
                "rerunning visual proof."
            )
            print_summary(results)
            if args.summary_json:
                write_summary_json(args.summary_json, results)
                print(f"JSON summary: {display_path(args.summary_json)}")
            return 1
    needs_isaac_startup = needs_isaac_gpu
    if needs_isaac_startup and not args.skip_isaac_startup_preflight:
        print()
        print("== host ==")
        result = run_isaac_startup_preflight(args, env, selected)
        if result is not None:
            results.append(result)
            if result.status == "failed":
                print(
                    "Isaac Sim startup preflight failed; skipping workspace "
                    "Isaac GPU checks. Verify SimulationApp startup before "
                    "rerunning proof."
                )
                print_summary(results)
                if args.summary_json:
                    write_summary_json(args.summary_json, results)
                    print(f"JSON summary: {display_path(args.summary_json)}")
                return 1

    for workspace in selected:
        print()
        print(f"== {workspace.name} ==")
        ran_down = False
        for check in checks:
            result = run_check(workspace, check, args, env)
            results.append(result)
            if check == "down":
                ran_down = True
            if result.status == "failed":
                if "down" in checks and check != "down" and not ran_down:
                    cleanup = run_check(workspace, "down", args, env)
                    results.append(cleanup)
                    ran_down = True
                if not args.continue_on_failure:
                    print_summary(results)
                    if args.summary_json:
                        write_summary_json(args.summary_json, results)
                    return 1
                break

    print_summary(results)
    if args.summary_json:
        write_summary_json(args.summary_json, results)
        print(f"JSON summary: {display_path(args.summary_json)}")
    return 1 if any(result.status == "failed" for result in results) else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        raise SystemExit(130)
