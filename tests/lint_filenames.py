import os
import glob

current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

# Check if all default files exist
DEFAULT_FILES = [
    ".gitignore",
    "README.md",
    "docker/.bashrc",
    "docker/.dockerignore",
    "docker/compose.yaml",
    "docker/Dockerfile",
    ".devcontainer/devcontainer.json",
]
for filename in DEFAULT_FILES:
    print(f"Checking existence of: '{filename}'...")
    for workspace_path in glob.glob(f"{repo_dir}/*_ws"):
        if not os.path.isfile(f"{workspace_path}/{filename}"):
            # Skip certain cases intentionally
            if filename in (".gitignore", "docker/.bashrc") and os.path.basename(workspace_path) == "ros1_bridge_ws" or \
               filename in ("docker/.bashrc") and os.path.basename(workspace_path) == "orbslam3_ws":
                continue
            # Report error
            raise ValueError(f"'{filename}' does not exist in: '{workspace_path}'")

# Check compose.yaml files
for filename in glob.glob(f"{repo_dir}/**/compose*.yaml", recursive=True):
    print(f"Checking: '{filename[len(repo_dir)+1:]}'...")
    with open(filename, "r") as f:
        content = f.read()
        if "version:" in content:
            # Ref: https://docs.docker.com/compose/compose-file/04-version-and-name/#version-top-level-element-optional
            raise ValueError(f"`version` should not exist since it's obsolete: '{filename}'")

# Check if all obsolete files do not exist
OBSOLETE_FILES = [
    "docker/cache/.gazebo/.gitkeep",
    "docker/compose.yml",
    "docker/docker-compose.yaml",
    "docker/docker-compose.yml",
    ".devcontainer/postCreateCommand.sh",
]
for filename in OBSOLETE_FILES:
    print(f"Checking non-existence of: '{filename}'...")
    for workspace_path in glob.glob(f"{repo_dir}/*_ws"):
        if os.path.isfile(f"{workspace_path}/{filename}"):
            raise ValueError(f"'{filename}' exists in: '{workspace_path}'")

# Check if `master` branch is accidentally used
for filename in glob.glob(f"{repo_dir}/.github/workflows/*.yaml", recursive=True):
    print(f"Checking: '{filename[len(repo_dir)+1:]}'...")
    with open(filename, "r") as f:
        content = f.read()
        if "master" in content:
            # Ref: https://github.com/j3soon/ros2-essentials/pull/44#pullrequestreview-2251404984
            raise ValueError(f"`master` should not exist since it's obsolete: '{filename}'")

print("All checks passed!")
