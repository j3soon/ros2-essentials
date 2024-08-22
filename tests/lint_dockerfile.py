import glob
import logging
import os
from pathlib import Path

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

logging.info("Checking `Dockerfile` files...")
for filename in glob.glob(f"{repo_dir}/*_ws/docker/Dockerfile*"):
    logging.debug(f"Checking: '{filename[len(repo_dir)+1:]}'...")
    # Skip certain cases intentionally
    if "ros1_bridge_ws" in filename:
        continue
    content = Path(filename).read_text()
    if " ros-humble-" in content:
        raise ValueError(f"` ros-humble-*` should not exist, use ` ros-$ROS_DISTRO-*` instead: '{filename}'")
    if "$USER_GID" in content:
        raise ValueError(f"`$USER_GID` should not exist since it's unnecessary: '{filename}'")
    if "RUN apt-get update" in content:
        raise ValueError(f"`RUN apt-get update` should not exist, use cache mounts instead: '{filename}'")
    if "RUN sudo apt-get update" in content:
        raise ValueError(f"`RUN sudo apt-get update` should not exist, use cache mounts instead: '{filename}'")
    if "RUN apt-get install" in content:
        raise ValueError(f"`RUN apt-get install` should not exist, use with `apt-get update` instead: '{filename}'")
    if "    apt-get install" in content:
        raise ValueError(f"`    apt-get install` should not exist, use with `apt-get update` in the same line instead: '{filename}'")
