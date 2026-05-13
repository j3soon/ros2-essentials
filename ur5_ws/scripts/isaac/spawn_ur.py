"""Spawn a UR5 or UR5e in Isaac Sim and step the physics loop.

Usage (inside the ur5_ws container):
    ${ISAACSIM_PYTHON_EXE} ur5_ws/scripts/isaac/spawn_ur.py --ur_type ur5
    ${ISAACSIM_PYTHON_EXE} ur5_ws/scripts/isaac/spawn_ur.py --ur_type ur5e --headless

The convenience wrappers ur5_ws/scripts/isaac_sim_ur5.sh and isaac_sim_ur5e.sh
just call this script with the appropriate --ur_type.
"""

import argparse

parser = argparse.ArgumentParser(description="Spawn a UR robot in Isaac Sim")
parser.add_argument("--ur_type", choices=["ur5", "ur5e"], default="ur5")
parser.add_argument("--headless", action="store_true")
parser.add_argument(
    "--steps",
    type=int,
    default=0,
    help="number of physics steps to run; 0 = until the window is closed",
)
args = parser.parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root = get_assets_root_path()
if assets_root is None:
    raise RuntimeError(
        "Could not resolve Isaac Sim assets root. "
        "Check Nucleus connectivity or set ISAAC_NUCLEUS_DIR."
    )

usd_path = f"{assets_root}/Isaac/Robots/UniversalRobots/{args.ur_type}/{args.ur_type}.usd"
prim_path = f"/World/{args.ur_type.upper()}"

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
world.reset()

step = 0
while simulation_app.is_running():
    world.step(render=True)
    step += 1
    if args.steps > 0 and step >= args.steps:
        break

simulation_app.close()
