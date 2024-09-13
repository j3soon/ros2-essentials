"""
Script for launching Isaac Sim in ROS2 launch files.

If Isaac Sim were installed through Omniverse Launcher or the official
docker image, we can simply follow the launch files guides below:
* https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_launch.html
* https://github.com/isaac-sim/IsaacSim-ros_workspaces/blob/main/humble_ws/src/isaacsim/scripts/open_isaacsim_stage.py

However, since we opt to use `pip install` instead, the guides above
aren't compatible. We simply integrate by ourselves.
"""

# TODO: This should be stored in an Isaac Sim package in `isaac_sim_ws`,
#       and be reused across workspaces.

import argparse

import omni.timeline
from omni.isaac.core.utils.stage import open_stage

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('--path', type=str, required=True, help='Path to the USD stage')
args = parser.parse_args()

# Load the USD stage from the provided argument
open_stage(args.path)

# Play the timeline
timeline = omni.timeline.get_timeline_interface()
timeline.play()
