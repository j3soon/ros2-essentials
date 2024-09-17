import glob
import logging
import os
from pathlib import Path

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

logging.info("Checking `.gitignore` files...")
for filename in glob.glob(f"{repo_dir}/*_ws/.gitignore"):
    # Global ignore
    if any(ws in filename for ws in os.getenv('IGNORED_WORKSPACES', '').split()):
        continue
    logging.debug(f"Checking: '{filename[len(repo_dir)+1:]}'...")
    content = Path(filename).read_text()
    if "gazebo" in content:
        raise ValueError(f"`gazebo` should not exist since it's obsolete: '{filename}'")
    if "docker/cache" in content:
        raise ValueError(f"`docker/cache` should not exist since it's obsolete: '{filename}'")
    if "PLACEHOLDER" in content:
        raise ValueError(f"`PLACEHOLDER` should not exist: '{filename}'")
