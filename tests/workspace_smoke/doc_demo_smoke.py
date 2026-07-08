#!/usr/bin/env python3
"""Run representative documented workspace demos and collect proof artifacts."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from proof_capture import capture_x11_screenshot, record_x11_video


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPORT_DIR = REPO_ROOT / "tests" / "workspace_smoke" / "artifacts" / "doc-demo"
ISAAC_GUI_SCRIPT = REPO_ROOT / "tests" / "workspace_smoke" / "isaac_gui_open_stage.py"
MIN_GUI_SCREENSHOT_BYTES = 10_000
MIN_GUI_RECORDING_BYTES = 100_000
COMPOSE_UP_LOCAL_ARGS = ["up", "-d", "--build", "--pull", "never"]
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
    joint_command: str | None = None
    pre_play_record_seconds: int = 3
    post_play_stable_seconds: int = 8
    recording_seconds: int = 24
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
        kind="joint_command",
        doc="docs/go2-ws/index.md#custom-isaac-sim-environment",
        command=(
            "open go2_og.usda in Isaac Sim, echo /clock and /joint_states, "
            "then publish the documented /joint_command"
        ),
        stage="/home/ros2-essentials/go2_ws/src/isaacsim/assets/go2_og.usda",
        expected_prim="/World/go2",
        joint_command="""ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{
  name: [
    'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
    'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
    'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint'
  ],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [],
  effort: []
}" """,
        settle_seconds=110,
        timeout_seconds=150,
    ),
    "h1_ws": Demo(
        workspace="h1_ws",
        kind="joint_command",
        doc="docs/h1-ws/index.md#custom-isaac-sim-environment",
        command=(
            "open h1_og.usda in Isaac Sim, echo /clock and /joint_states, "
            "then publish the documented /joint_command"
        ),
        stage="/home/ros2-essentials/h1_ws/isaacsim/assets/h1_og.usda",
        expected_prim="/World/h1",
        joint_command="""ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{
  name: [
    'left_hip_yaw_joint',
    'right_hip_yaw_joint',
    'torso_joint',
    'left_hip_roll_joint',
    'right_hip_roll_joint',
    'left_shoulder_pitch_joint',
    'right_shoulder_pitch_joint',
    'left_hip_pitch_joint',
    'right_hip_pitch_joint',
    'left_shoulder_roll_joint',
    'right_shoulder_roll_joint',
    'left_knee_joint',
    'right_knee_joint',
    'left_shoulder_yaw_joint',
    'right_shoulder_yaw_joint',
    'left_ankle_joint',
    'right_ankle_joint',
    'left_elbow_joint',
    'right_elbow_joint'
  ],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [],
  effort: []
}" """,
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
    artifact_paths: tuple[Path, ...] = ()


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


def artifact_timestamp(path: Path) -> str:
    return path.name.split("-", 1)[0]


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


def no_registry_cache_override_path(report_dir: Path, demo: Demo) -> Path:
    override_path = report_dir / demo.workspace / "no-registry-cache.compose.yaml"
    override_path.parent.mkdir(parents=True, exist_ok=True)
    override_path.write_text(
        "\n".join(
            [
                "services:",
                f"  {compose_service(demo)}:",
                "    build:",
                "      cache_from: !reset []",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return override_path


def compose_command(demo: Demo, args: list[str], override_files: tuple[Path, ...] = ()) -> list[str]:
    command = ["docker", "compose"]
    for index, override_file in enumerate(override_files):
        if index == 0:
            command.extend(["-f", "compose.yaml"])
        command.extend(["-f", str(override_file)])
    command.extend(args)
    return command


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


def append_command(command: list[str], log_path: Path, *, cwd: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"$ {' '.join(command)}")
    print(f"log: {display_path(log_path)}")
    with log_path.open("a", encoding="utf-8") as log_file:
        log_file.write(f"$ {' '.join(command)}\n")
        log_file.flush()
        result = subprocess.run(command, cwd=cwd, stdout=log_file, stderr=subprocess.STDOUT, text=True)
        log_file.write(f"[exit {result.returncode}] {' '.join(command)}\n")
        return result.returncode


def compose_exec_shell(demo: Demo, shell_command: str, *, tty: bool = False) -> list[str]:
    command = ["docker", "compose", "exec"]
    if not tty:
        command.append("-T")
    command.extend([compose_service(demo), "bash", "-lc", shell_command])
    return command


def ros_shell(demo: Demo, command: str) -> str:
    workspace_path = shlex.quote(f"/home/ros2-essentials/{demo.workspace}")
    return (
        "set -o pipefail; "
        "source /home/user/.bashrc; "
        f"cd {workspace_path}; "
        "if [ -f install/setup.bash ]; then source install/setup.bash; fi; "
        f"{command}"
    )


def wait_for_log_markers(log_path: Path, markers: tuple[str, ...], timeout_seconds: int) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if log_path.is_file():
            text = log_path.read_text(encoding="utf-8", errors="replace")
            if all(marker in text for marker in markers):
                return True
        time.sleep(1)
    return False


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
    override_files = (no_registry_cache_override_path(args.report_dir, demo),)
    play_trigger_path = f"/tmp/{demo.workspace}-{artifact_timestamp(base)}-play.trigger"
    env = {
        "DISPLAY": args.display,
        "XAUTHORITY": "/home/user/.Xauthority",
        "ISAAC_GUI_STAGE_PATH": demo.stage,
        "ISAAC_GUI_PLAY": "false",
        "ISAAC_GUI_PLAY_TRIGGER_PATH": play_trigger_path,
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
        up_command = compose_command(demo, COMPOSE_UP_LOCAL_ARGS, override_files)
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


def run_joint_command(demo: Demo, args: argparse.Namespace) -> Result:
    assert demo.stage is not None
    assert demo.joint_command is not None
    base = artifact_base(args.report_dir, demo.workspace, "doc-joint-command")
    log_path = base.with_suffix(".log")
    screenshot_path = base.with_suffix(".png")
    recording_path = base.with_suffix(".mp4")
    override_files = (no_registry_cache_override_path(args.report_dir, demo),)
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
    isaac_command = ["script", "-qefc", " ".join(shlex.quote(part) for part in exec_command), "/dev/null"]
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"$ {' '.join(isaac_command)}")
    print(f"log: {display_path(log_path)}")
    with log_path.open("w", encoding="utf-8") as log_file:
        up_command = compose_command(demo, COMPOSE_UP_LOCAL_ARGS, override_files)
        log_file.write(f"$ {' '.join(up_command)}\n")
        up_result = subprocess.run(
            up_command,
            cwd=compose_dir(demo),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )
        log_file.write(f"[exit {up_result.returncode}] {' '.join(up_command)}\n")
        log_file.write(f"$ {' '.join(isaac_command)}\n")
        log_file.flush()
        if up_result.returncode != 0:
            return Result(
                demo.workspace,
                demo.kind,
                "failed",
                demo.command,
                demo.doc,
                log_path,
                reason="docker compose up failed",
            )
        process = subprocess.Popen(
            isaac_command,
            cwd=compose_dir(demo),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )

    artifacts: list[Path] = []
    recording_result: list[int | None] = [None]
    recording_thread: threading.Thread | None = None
    try:
        ready_markers = (
            "ISAAC_GUI_STAGE_OPENED",
            "ISAAC_GUI_EXPECTED_PRIM_OK",
            "ISAAC_GUI_WAITING_FOR_PLAY_TRIGGER",
        )
        ready = wait_for_log_markers(log_path, ready_markers, demo.timeout_seconds)
        if not ready:
            capture_x11(screenshot_path, args)
            artifacts.append(screenshot_path)
            return Result(
                demo.workspace,
                demo.kind,
                "failed",
                demo.command,
                demo.doc,
                log_path,
                screenshot_path if screenshot_path.is_file() else None,
                f"Isaac GUI did not reach {demo.workspace} stage readiness markers",
                artifact_paths=tuple(path for path in artifacts if path.is_file()),
            )

        def record_motion() -> None:
            recording_result[0] = record_x11_video(
                recording_path,
                display=args.display,
                x11_size=args.x11_size,
                seconds=demo.recording_seconds,
                framerate=15,
            )

        recording_thread = threading.Thread(target=record_motion)
        recording_thread.start()
        with log_path.open("a", encoding="utf-8") as log_file:
            log_file.write(
                "Recording started before Isaac timeline play. "
                f"Pre-play capture: {demo.pre_play_record_seconds}s\n"
            )
        time.sleep(demo.pre_play_record_seconds)
        append_command(
            compose_exec_shell(demo, f"touch {shlex.quote(play_trigger_path)}"),
            log_path,
            cwd=compose_dir(demo),
        )
        timeline_ready = wait_for_log_markers(log_path, ("ISAAC_GUI_TIMELINE_PLAYING",), 45)

        topic_checks = [
            (
                "/clock",
                "timeout --preserve-status 35s ros2 topic echo --once /clock",
            ),
            (
                "/joint_states",
                "timeout --preserve-status 35s ros2 topic echo --once /joint_states",
            ),
        ]
        topic_failures = []
        if timeline_ready:
            for topic_name, topic_command in topic_checks:
                code = append_command(compose_exec_shell(demo, ros_shell(demo, topic_command)), log_path, cwd=compose_dir(demo))
                if code != 0:
                    topic_failures.append(topic_name)
            with log_path.open("a", encoding="utf-8") as log_file:
                log_file.write(
                    f"Scene stabilization after timeline play before /joint_command: {demo.post_play_stable_seconds}s\n"
                )
            time.sleep(demo.post_play_stable_seconds)
        else:
            topic_failures.append("timeline_play")

        publish_code = append_command(compose_exec_shell(demo, ros_shell(demo, demo.joint_command)), log_path, cwd=compose_dir(demo))
        recording_thread.join(timeout=demo.recording_seconds + 10)
        capture_x11(screenshot_path, args)
        for path in (screenshot_path, recording_path):
            if path.is_file():
                artifacts.append(path)
    finally:
        with log_path.open("a", encoding="utf-8") as log_file:
            if recording_thread and recording_thread.is_alive():
                log_file.write("recording thread did not finish before cleanup\n")
            log_file.write("$ docker compose down --remove-orphans\n")
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
            try:
                code = process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                code = process.wait(timeout=30)

    text = log_path.read_text(encoding="utf-8", errors="replace")
    screenshot_ok = screenshot_path.is_file() and screenshot_path.stat().st_size >= MIN_GUI_SCREENSHOT_BYTES
    recording_ok = recording_path.is_file() and recording_path.stat().st_size >= MIN_GUI_RECORDING_BYTES
    stage_ok = "ISAAC_GUI_STAGE_OPENED" in text
    prim_ok = "ISAAC_GUI_EXPECTED_PRIM_OK" in text
    timeline_ok = "ISAAC_GUI_TIMELINE_PLAYING" in text
    reason = None
    if not stage_ok:
        reason = f"Isaac GUI did not report {demo.workspace} stage opened"
    elif not prim_ok:
        reason = f"Isaac GUI did not report expected prim: {demo.expected_prim}"
    elif not timeline_ok:
        reason = "Isaac GUI did not report timeline playing"
    elif topic_failures:
        reason = f"ROS topic echo failed for: {', '.join(topic_failures)}"
    elif publish_code != 0:
        reason = f"/joint_command publish exited with {publish_code}"
    elif not screenshot_ok:
        reason = f"{demo.workspace} final screenshot was not captured or appears blank"
    elif not recording_ok:
        reason = f"{demo.workspace} motion recording was not captured or appears too small"
    elif recording_result[0] != 0:
        reason = f"{demo.workspace} motion recording exited with {recording_result[0]}"
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
        screenshot_path if screenshot_path.is_file() else None,
        reason,
        artifact_paths=tuple(path for path in artifacts if path.is_file()),
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
        process = subprocess.Popen(command, cwd=REPO_ROOT, text=True, stdout=log_file, stderr=subprocess.STDOUT)
        time.sleep(demo.settle_seconds)
        capture_x11(output_path, args)
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
    if demo.kind == "joint_command":
        return run_joint_command(demo, args)
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
        artifact_paths = result.artifact_paths or ((result.artifact_path,) if result.artifact_path else ())
        if len(artifact_paths) == 1:
            detail += f" artifact={display_path(artifact_paths[0])}"
        elif artifact_paths:
            detail += " artifacts=" + ",".join(display_path(path) or "" for path in artifact_paths)
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
                "artifact_paths": [
                    display_path(artifact)
                    for artifact in (result.artifact_paths or ((result.artifact_path,) if result.artifact_path else ()))
                ],
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
