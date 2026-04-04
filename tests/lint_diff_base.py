import logging
import os
from pathlib import Path

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

compose_path = Path(f"{repo_dir}/tests/diff_base/docker/compose.yaml")
content = compose_path.read_text()

logging.info("Checking `tests/diff_base/docker/compose.yaml` placeholders...")

required_lines = [
    '{PLACEHOLDER_#}CUDA_TOOLKIT_VERSION: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}ISAAC_SIM_VERSION: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}ISAAC_LAB_VERSION: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}ISAAC_ROS: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}CARTOGRAPHER: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}RTABMAP: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}REALSENSE: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}NV_OPENUSD: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}NEWTON_TOOLS: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}CLAUDE_CODE: "{PLACEHOLDER}"',
    '{PLACEHOLDER_#}CODEX: "{PLACEHOLDER}"',
    "{PLACEHOLDER_#}privileged: true",
    "{PLACEHOLDER_#}- /dev/dri:/dev/dri",
    "{PLACEHOLDER_#}- /dev/snd:/dev/snd",
    "{PLACEHOLDER_#}- /dev/shm:/dev/shm",
    "{PLACEHOLDER_#}- /dev:/dev",
]

for line in required_lines:
    if line not in content:
        raise ValueError(f"Missing required placeholder line in '{compose_path}': {line}")
