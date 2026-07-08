#!/usr/bin/env python3
"""Run representative documented workspace demos and collect proof artifacts."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from proof_capture import capture_x11_screenshot


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPORT_DIR = REPO_ROOT / "tests" / "workspace_smoke" / "artifacts" / "doc-demo"
ISAAC_GUI_SCRIPT = REPO_ROOT / "tests" / "workspace_smoke" / "isaac_gui_open_stage.py"
MIN_GUI_SCREENSHOT_BYTES = 10_000
GUI_FAILURE_MARKERS = (
    "[ERROR]",
    "Caught exception",
    "Package '",
    "file not found",
    "process has died",
    "Service /spawn_entity unavailable",
)


@dataclass(frozen=True)
class Demo:
    workspace: str
    kind: str
    command: str
    doc: str
    expected: tuple[str, ...] = ()
    env: dict[str, str] = field(default_factory=dict)
    stage: str | None = None
    expected_prim: str | None = None
    reason: str | None = None
    settle_seconds: int = 18
    timeout_seconds: int = 45

    @property
    def image(self) -> str:
        return f"j3soon/ros2-{self.workspace.replace('_', '-')}"


DEMOS: dict[str, Demo] = {
    "aloha_ws": Demo(
        workspace="aloha_ws",
        kind="gui",
        doc="docs/aloha-ws/index.md#view-robot-model-in-rviz",
        command=(
            "ros2 launch interbotix_xsarm_descriptions "
            "xsarm_description.launch.py robot_model:=vx300s use_joint_pub_gui:=true"
        ),
        expected=("robot_state_publisher", "rviz2"),
    ),
    "delto_gripper_ws": Demo(
        workspace="delto_gripper_ws",
        kind="isaac_stage",
        doc="docs/delto-gripper-ws/index.md#launch-the-isaacsim",
        command=(
            "~/isaacsim/isaac-sim.sh and open "
            "delto_gripper_ws/src/DELTO_M_ROS2/dg_isaacsim/dg5f_right/dg5f_right.usd"
        ),
        stage=(
            "/home/ros2-essentials/delto_gripper_ws/src/DELTO_M_ROS2/"
            "dg_isaacsim/dg5f_right/dg5f_right.usda"
        ),
        expected_prim="/dg5f_right",
        settle_seconds=110,
        timeout_seconds=150,
    ),
    "gazebo_world_ws": Demo(
        workspace="gazebo_world_ws",
        kind="gui",
        doc="docs/gazebo-world-ws/index.md#use-in-gazebo_world_ws-container",
        command=(
            "ros2 launch gazebo_launch turtlebot3.launch.py "
            "gazebo_world:=turtlebot3_dqn_stage3.world"
        ),
        expected=("gzserver", "gzclient"),
        settle_seconds=25,
        timeout_seconds=60,
    ),
    "go2_ws": Demo(
        workspace="go2_ws",
        kind="isaac_stage",
        doc="docs/go2-ws/index.md#custom-isaac-sim-environment",
        command=(
            "~/isaacsim/isaac-sim.sh and open "
            "/home/ros2-essentials/go2_ws/isaacsim/assets/go2_og.usda"
        ),
        stage="/home/ros2-essentials/go2_ws/src/isaacsim/assets/go2_og.usda",
        expected_prim="/World/go2",
        settle_seconds=110,
        timeout_seconds=150,
    ),
    "h1_ws": Demo(
        workspace="h1_ws",
        kind="isaac_stage",
        doc="docs/h1-ws/index.md#custom-isaac-sim-environment",
        command=(
            "~/isaacsim/isaac-sim.sh and open "
            "/home/ros2-essentials/h1_ws/isaacsim/assets/h1_og.usda"
        ),
        stage="/home/ros2-essentials/h1_ws/isaacsim/assets/h1_og.usda",
        expected_prim="/World/h1",
        settle_seconds=110,
        timeout_seconds=150,
    ),
    "husky_ws": Demo(
        workspace="husky_ws",
        kind="gui",
        doc="docs/husky-ws/index.md#view-the-model",
        command="ros2 launch husky_viz view_model_launch.py",
        expected=("robot_state_publisher", "rviz2"),
    ),
    "kobuki_ws": Demo(
        workspace="kobuki_ws",
        kind="gui",
        doc="docs/kobuki-ws/index.md#visualize-the-model-in-rviz",
        command="ros2 launch kobuki_rviz view_model_launch.py",
        expected=("robot_state_publisher", "rviz2"),
    ),
    "orbslam3_ws": Demo(
        workspace="orbslam3_ws",
        kind="skip",
        doc="docs/orbslam3-ws/index.md#simple-test-with-dataset",
        command=(
            "ros2 bag play V1_02_medium/... and ros2 run orbslam3 mono "
            "~/ORB_SLAM3/Vocabulary/ORBvoc.txt ..."
        ),
        reason="requires external EuRoC dataset download and bag playback",
    ),
    "ros1_bridge_ws": Demo(
        workspace="ros1_bridge_ws",
        kind="skip",
        doc="docs/ros1-bridge-ws/index.md#run-the-bridge-and-the-example-talker-and-listener",
        command="docker compose up, ROS1 talker/listener, ROS2 talker/listener",
        reason="multi-service ROS1 bridge validation is tracked separately and has no screenshot surface",
    ),
    "so101_ws": Demo(
        workspace="so101_ws",
        kind="skip",
        doc="docs/so101-ws/index.md#hardware-testing",
        command="LeRobot hardware calibration/teleoperation and LeIsaac teleoperation",
        reason="documented flows require real SO-101 hardware, camera, or interactive teleoperation",
    ),
    "stretch3_ws": Demo(
        workspace="stretch3_ws",
        kind="isaac_stage",
        doc="docs/stretch3-ws/index.md#isaac-sim-keyboard-control",
        command=(
            "~/isaacsim/isaac-sim.sh and open "
            "/home/ros2-essentials/stretch3_ws/isaacsim/assets/stretch3_og_wasd.usda"
        ),
        stage="/home/ros2-essentials/stretch3_ws/isaacsim/assets/stretch3_og_wasd.usda",
        expected_prim="/World/stretch3",
        settle_seconds=115,
        timeout_seconds=160,
    ),
    "template_ws": Demo(
        workspace="template_ws",
        kind="skip",
        doc="docs/template-ws/index.md",
        command="template instructions only",
        reason="template workspace does not document a workspace-specific simulator demo",
    ),
    "turtlebot3_ws": Demo(
        workspace="turtlebot3_ws",
        kind="gui",
        doc="docs/turtlebot3-ws/index.md#rviz-fake-node",
        command="ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py",
        expected=("Turtlebot3 fake node has been initialised", "rviz2"),
        env={"TURTLEBOT3_MODEL": "burger"},
    ),
    "ur5_ws": Demo(
        workspace="ur5_ws",
        kind="skip",
        doc="docs/ur5-ws/index.md#simulation-with-ursim-cb3",
        command="URSim CB3 Docker plus ur_robot_driver / MoveIt launch files",
        reason="documented simulator depends on a separate URSim container and robot calibration parameters",
    ),
    "vlp_ws": Demo(
        workspace="vlp_ws",
        kind="gui",
        doc="docs/vlp-ws/index.md#launch-lidar-driver-with-simulated-lidar",
        command="ros2 launch velodyne_description example.launch.py",
        expected=("robot_state_publisher", "rviz2"),
        settle_seconds=20,
        timeout_seconds=50,
    ),
}


@dataclass
class Result:
    workspace: str
    kind: str
    status: str
    command: str
    doc: str
    log_path: Path | None = None
    artifact_path: Path | None = None
    reason: str | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", action="append", default=[], choices=sorted(DEMOS))
    parser.add_argument("--all", action="store_true", help="Run every configured workspace demo.")
    parser.add_argument("--list", action="store_true", help="List configured demo workspaces.")
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT_DIR)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument("--x11-size", default="1280x720")
    parser.add_argument(
        "--image-override",
        action="append",
        default=[],
        metavar="WORKSPACE=IMAGE",
        help="Use IMAGE for one workspace instead of the default j3soon/ros2-* tag.",
    )
    return parser.parse_args()


def display_path(path: Path | None) -> str | None:
    if path is None:
        return None
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def selected_demos(args: argparse.Namespace) -> list[Demo]:
    if args.list:
        return []
    names = sorted(DEMOS) if args.all else sorted(set(args.workspace))
    if not names:
        raise ValueError("Select --all or at least one --workspace.")
    return [DEMOS[name] for name in names]


def artifact_base(report_dir: Path, workspace: str, kind: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return report_dir / workspace / f"{timestamp}-{kind}"


def image_overrides(args: argparse.Namespace) -> dict[str, str]:
    overrides = {}
    for override in args.image_override:
        workspace, separator, image = override.partition("=")
        if not separator or not workspace or not image:
            raise ValueError("--image-override values must use WORKSPACE=IMAGE")
        if workspace not in DEMOS:
            raise ValueError(f"unknown workspace in --image-override: {workspace}")
        overrides[workspace] = image
    return overrides


def image_for(demo: Demo, args: argparse.Namespace) -> str:
    return image_overrides(args).get(demo.workspace, demo.image)


def compose_service(demo: Demo) -> str:
    return demo.workspace.replace("_", "-")


def compose_dir(demo: Demo) -> Path:
    return REPO_ROOT / demo.workspace / "docker"


def run_command(command: list[str], log_path: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"$ {' '.join(command)}")
    print(f"log: {display_path(log_path)}")
    with log_path.open("w", encoding="utf-8") as log_file:
        log_file.write(f"$ {' '.join(command)}\n")
        process = subprocess.Popen(command, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="")
            log_file.write(line)
        return process.wait()


def docker_base(demo: Demo, args: argparse.Namespace) -> list[str]:
    env = {
        "DISPLAY": args.display,
        "ROS_LOCALHOST_ONLY": "0",
        "ROS_DOMAIN_ID": "51",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS2_WS": f"/home/ros2-essentials/{demo.workspace}",
        **demo.env,
    }
    command = [
        "docker",
        "run",
        "--rm",
        "--network",
        "host",
        "--privileged",
        "--gpus",
        "all",
    ]
    for key, value in env.items():
        command.extend(["-e", f"{key}={value}"])
    command.extend(
        [
            "-v",
            "/tmp/.X11-unix:/tmp/.X11-unix",
            "-v",
            f"{Path.home() / '.Xauthority'}:/home/user/.Xauthority:ro",
            "-v",
            f"{REPO_ROOT}:/home/ros2-essentials",
            "-w",
            f"/home/ros2-essentials/{demo.workspace}",
            "--entrypoint",
            "bash",
            image_for(demo, args),
            "--noprofile",
            "--norc",
        ]
    )
    return command


def ros2_launch_package(command: str) -> str | None:
    parts = shlex.split(command)
    for index, part in enumerate(parts):
        if part == "ros2" and parts[index + 1 : index + 2] == ["launch"]:
            package_index = index + 2
            if package_index < len(parts):
                return parts[package_index]
    return None


def run_isaac_stage(demo: Demo, args: argparse.Namespace) -> Result:
    assert demo.stage is not None
    base = artifact_base(args.report_dir, demo.workspace, "doc-isaac-stage")
    log_path = base.with_suffix(".log")
    output_path = base.with_suffix(".png")
    env = {
        "DISPLAY": args.display,
        "XAUTHORITY": "/home/user/.Xauthority",
        "ISAAC_GUI_STAGE_PATH": demo.stage,
        "ISAAC_GUI_PLAY": "true",
        "ISAAC_GUI_SETTLE_FRAMES": "180",
    }
    if demo.expected_prim:
        env["ISAAC_GUI_EXPECTED_PRIM"] = demo.expected_prim
    exec_command = [
        "docker",
        "compose",
        "exec",
        *[item for key, value in env.items() for item in ("-e", f"{key}={value}")],
        compose_service(demo),
        "bash",
        "-lc",
        "/home/user/isaacsim/isaac-sim.sh --exec "
        "/home/ros2-essentials/tests/workspace_smoke/isaac_gui_open_stage.py",
    ]
    command = ["script", "-qefc", " ".join(shlex.quote(part) for part in exec_command), "/dev/null"]
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"$ {' '.join(command)}")
    print(f"log: {display_path(log_path)}")
    with log_path.open("w", encoding="utf-8") as log_file:
        up_command = ["docker", "compose", "up", "-d"]
        log_file.write(f"$ {' '.join(up_command)}\n")
        subprocess.run(up_command, cwd=compose_dir(demo), stdout=log_file, stderr=subprocess.STDOUT, text=True)
        log_file.write(f"$ {' '.join(command)}\n")
        process = subprocess.Popen(command, cwd=compose_dir(demo), stdout=log_file, stderr=subprocess.STDOUT, text=True)
        time.sleep(demo.settle_seconds)
        capture_x11(output_path, args)
        subprocess.run(
            ["docker", "compose", "down", "--remove-orphans"],
            cwd=compose_dir(demo),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            code = process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.terminate()
            code = process.wait(timeout=30)

    text = log_path.read_text(encoding="utf-8", errors="replace")
    artifact_ok = output_path.is_file() and output_path.stat().st_size >= MIN_GUI_SCREENSHOT_BYTES
    stage_ok = "ISAAC_GUI_STAGE_OPENED" in text
    prim_ok = demo.expected_prim is None or "ISAAC_GUI_EXPECTED_PRIM_OK" in text
    reason = None
    if not artifact_ok:
        reason = "Isaac GUI screenshot was not captured or appears blank"
    elif not stage_ok:
        reason = "Isaac GUI did not report stage opened"
    elif not prim_ok:
        reason = f"Isaac GUI did not report expected prim: {demo.expected_prim}"
    elif code not in {0, 124, 130, 137, 143}:
        reason = f"Isaac GUI container exited with {code}"
    status = "passed" if reason is None else "failed"
    return Result(
        demo.workspace,
        demo.kind,
        status,
        demo.command,
        demo.doc,
        log_path,
        output_path if output_path.is_file() else None,
        reason,
    )


def capture_x11(path: Path, args: argparse.Namespace) -> int:
    return capture_x11_screenshot(path, display=args.display, x11_size=args.x11_size)


def run_gui(demo: Demo, args: argparse.Namespace) -> Result:
    base = artifact_base(args.report_dir, demo.workspace, "doc-gui")
    log_path = base.with_suffix(".log")
    output_path = base.with_suffix(".png")
    launch_package = ros2_launch_package(demo.command)
    package_setup = ""
    if launch_package:
        quoted_package = shlex.quote(launch_package)
        package_setup = (
            f"if ! ros2 pkg prefix {quoted_package} >/dev/null 2>&1; then "
            "cd \"$ROS2_WS\"; "
            f"colcon build --symlink-install --packages-up-to {quoted_package}; "
            "source \"$ROS2_WS/install/setup.bash\"; "
            "fi; "
            f"ros2 pkg prefix {quoted_package}; "
        )
    shell_command = (
        "set -e; "
        "source /home/user/.bashrc; "
        f"{package_setup}"
        f"timeout --preserve-status {demo.timeout_seconds:d}s {demo.command}"
    )
    command = [*docker_base(demo, args), "-lc", shell_command]

    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"$ {' '.join(command)}")
    print(f"log: {display_path(log_path)}")
    with log_path.open("w", encoding="utf-8") as log_file:
        log_file.write(f"$ {' '.join(command)}\n")
        process = subprocess.Popen(command, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(demo.settle_seconds)
        capture_x11(output_path, args)
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="")
            log_file.write(line)
        code = process.wait()

    text = log_path.read_text(encoding="utf-8", errors="replace")
    expected_ok = all(pattern in text for pattern in demo.expected)
    timeout_ok = code in {0, 124, 130, 137, 143}
    failure_marker = next((marker for marker in GUI_FAILURE_MARKERS if marker in text), None)
    artifact_ok = output_path.is_file() and output_path.stat().st_size >= MIN_GUI_SCREENSHOT_BYTES
    reason = None
    if not timeout_ok:
        reason = f"demo command exited with {code}"
    elif not expected_ok:
        missing = [pattern for pattern in demo.expected if pattern not in text]
        reason = f"missing expected log markers: {', '.join(missing)}"
    elif failure_marker:
        reason = f"log contains failure marker: {failure_marker}"
    elif not artifact_ok:
        reason = "GUI screenshot was not captured or appears blank"
    status = "passed" if reason is None else "failed"
    return Result(
        demo.workspace,
        demo.kind,
        status,
        demo.command,
        demo.doc,
        log_path,
        output_path if output_path.is_file() else None,
        reason,
    )


def run_demo(demo: Demo, args: argparse.Namespace) -> Result:
    if demo.kind == "skip":
        print(f"[SKIP] {demo.workspace}: {demo.reason}")
        return Result(demo.workspace, demo.kind, "skipped", demo.command, demo.doc, reason=demo.reason)
    if demo.kind == "isaac_stage":
        return run_isaac_stage(demo, args)
    if demo.kind == "gui":
        return run_gui(demo, args)
    raise ValueError(f"Unsupported demo kind: {demo.kind}")


def print_summary(results: list[Result]) -> None:
    print()
    print("Summary:")
    for result in results:
        detail = ""
        if result.log_path:
            detail += f" log={display_path(result.log_path)}"
        if result.artifact_path:
            detail += f" artifact={display_path(result.artifact_path)}"
        if result.reason:
            detail += f" reason={result.reason}"
        print(f"  {result.workspace}: {result.kind}: {result.status}{detail}")


def write_summary(path: Path, results: list[Result]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "results": [
            {
                "workspace": result.workspace,
                "kind": result.kind,
                "status": result.status,
                "command": result.command,
                "doc": result.doc,
                "log_path": display_path(result.log_path),
                "artifact_path": display_path(result.artifact_path),
                "reason": result.reason,
            }
            for result in results
        ],
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.list:
        for name, demo in sorted(DEMOS.items()):
            print(f"{name}\t{demo.kind}\t{demo.doc}")
        return 0

    demos = selected_demos(args)
    results: list[Result] = []
    for demo in demos:
        print()
        print(f"== {demo.workspace} ==")
        result = run_demo(demo, args)
        results.append(result)
        if result.status == "failed" and not args.continue_on_failure:
            break

    print_summary(results)
    if args.summary_json:
        write_summary(args.summary_json, results)
        print(f"JSON summary: {display_path(args.summary_json)}")
    return 1 if any(result.status == "failed" for result in results) else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
