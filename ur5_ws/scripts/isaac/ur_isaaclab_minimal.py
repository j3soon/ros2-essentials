"""Minimal Isaac Lab scene with a UR5 or UR5e articulation.

Usage (inside the ur5_ws container):
    ~/IsaacLab/isaaclab.sh -p ur5_ws/scripts/isaac/ur_isaaclab_minimal.py --ur_type ur5e

This is a scaffold: it loads the chosen UR USD into an Isaac Lab scene,
applies small random joint targets, and steps the simulator. Use it as a
starting point for Reach / Pick / RL tasks by swapping the USD path and
joint names into Isaac Lab's manager-based task templates under
isaaclab_tasks.manager_based.manipulation.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Minimal UR articulation in Isaac Lab")
parser.add_argument("--ur_type", choices=["ur5", "ur5e"], default="ur5")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument(
    "--steps",
    type=int,
    default=0,
    help="number of sim steps to run; 0 = until the window is closed or Ctrl-C",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaacsim.storage.native import get_assets_root_path

assets_root = get_assets_root_path()
if assets_root is None:
    raise RuntimeError("Could not resolve Isaac Sim assets root.")

usd_path = f"{assets_root}/Isaac/Robots/UniversalRobots/{args.ur_type}/{args.ur_type}.usd"

UR_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": -1.57,
            "wrist_2_joint": -1.57,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"], stiffness=400.0, damping=40.0
        ),
    },
)


@configclass
class URSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)),
    )
    robot: ArticulationCfg = UR_CFG


sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 120.0)
sim = sim_utils.SimulationContext(sim_cfg)
sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

scene = InteractiveScene(URSceneCfg(num_envs=args.num_envs, env_spacing=2.0))
sim.reset()

robot: Articulation = scene["robot"]
default_joint_pos = robot.data.default_joint_pos.clone()

step = 0
while simulation_app.is_running():
    target = default_joint_pos + 0.2 * torch.sin(
        torch.tensor(step * 0.01, device=sim.device)
    )
    robot.set_joint_position_target(target)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())
    step += 1
    if args.steps > 0 and step >= args.steps:
        break

simulation_app.close()
