import os
import glob

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
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")
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
            raise ValueError(f"version should not exist since it's obsolete: '{filename}'")

print("All checks passed!")
