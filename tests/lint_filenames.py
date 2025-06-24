import glob
import logging
import os

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

logging.info("Checking if all default files exist...")
DEFAULT_FILES = [
    ".gitignore",
    "README.md",
    "docker/.bashrc",
    "docker/.dockerignore",
    "docker/compose.yaml",
    "docker/Dockerfile",
    "docker/cyclonedds.xml",
    ".devcontainer/devcontainer.json",
]
for filename in DEFAULT_FILES:
    logging.debug(f"Checking existence of: '{filename}'...")
    for workspace_path in glob.glob(f"{repo_dir}/*_ws"):
        # Global ignore
        if any(ws in workspace_path for ws in os.getenv('IGNORED_WORKSPACES', '').split()):
            continue
        if not os.path.isfile(f"{workspace_path}/{filename}"):
            # Skip certain cases intentionally
            if filename in (".gitignore", "docker/.bashrc", "docker/cyclonedds.xml") and os.path.basename(workspace_path) == "ros1_bridge_ws":
                continue
            # Report error
            raise ValueError(f"'{filename}' does not exist in: '{workspace_path}'")

logging.info("Checking if all obsolete files do not exist...")
OBSOLETE_FILES = [
    "docker/cache/.gazebo/.gitkeep",
    "docker/compose.yml",
    "docker/docker-compose.yaml",
    "docker/docker-compose.yml",
    ".devcontainer/postCreateCommand.sh",
]
for filename in OBSOLETE_FILES:
    logging.debug(f"Checking non-existence of: '{filename}'...")
    for workspace_path in glob.glob(f"{repo_dir}/*_ws"):
        # Global ignore
        if any(ws in workspace_path for ws in os.getenv('IGNORED_WORKSPACES', '').split()):
            continue
        if os.path.isfile(f"{workspace_path}/{filename}"):
            raise ValueError(f"'{filename}' exists in: '{workspace_path}'")
