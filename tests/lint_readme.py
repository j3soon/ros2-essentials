import filecmp
import glob
import logging
import os
from pathlib import Path

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

logging.info("Checking `README.md` files...")
for filename in glob.glob(f"{repo_dir}/*_ws/README.md"):
    # Global ignore
    if any(ws in filename for ws in os.getenv('IGNORED_WORKSPACES', '').split()):
        continue
    logging.debug(f"Checking: '{filename[len(repo_dir)+1:]}'...")
    content = Path(filename).read_text()
    if "ros2-agv-essentials" in content:
        raise ValueError(f"`ros2-agv-essentials` should not exist, use `ros2-essentials` instead: '{filename}'")
    if "PLACEHOLDER" in content:
        raise ValueError(f"`PLACEHOLDER` should not exist: '{filename}'")

if not filecmp.cmp(f"{repo_dir}/README.md", f"{repo_dir}/docs/index.md"):
    raise ValueError(f"`README.md` should be the same as `docs/index.md`")
