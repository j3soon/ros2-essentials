# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html
# launch Isaac Sim before any other imports
# default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html#importing-urdf-using-python

import logging
import os

import omni.kit.commands
import omni.usd
from omni.importer.urdf import _urdf
from pxr import UsdPhysics

# Ref: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/logging.html
# `print(...)` outputs to Script Editor, while `logger.info(...)` outputs to console.
logger = logging.getLogger(__name__)


def create_vx300s_from_urdf(urdf_path, usd_path):
    # Set the settings in the import config
    import_config = _urdf.ImportConfig()
    import_config.make_default_prim = True
    # Finally import the robot & save it as USD
    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=urdf_path,
        import_config=import_config, dest_path=usd_path,
    )
    omni.usd.get_context().open_stage(usd_path)
    stage = omni.usd.get_context().get_stage()
    # Articulation Root for a fixed-base articulation can be either:
    # (1) the fixed joint that connects the articulation base to the world, or
    # (2) an ancestor of the fixed joint in the USD hierarchy.
    # Ref: https://docs.omniverse.nvidia.com/extensions/latest/ext_physics/articulations.html#articulationroot
    # The URDF Importer uses the first option to define articulation root for the imported robot.
    # Unfortunately, the first option somehow reports a false-positive error when using OmniGraph.
    # The false-positive error message doesn't affect the actual functionalities, but may bring potential confusions.
    # Therefore, we opt to manually switch to using the second option.
    # TODO: Switch back to the first option after this issue is fixed.
    prim = stage.GetPrimAtPath("/vx300s")
    # Note that this somehow cannot be applied through GUI, and must be done through Python
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    prim = stage.GetPrimAtPath("/vx300s/root_joint")
    prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    omni.usd.get_context().save_stage()

if __name__ == '__main__':
    vx300s_urdf_path = f'{os.path.expanduser("~")}/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/vx300s.urdf'
    vx300s_usd_path = '/home/ros2-essentials/aloha_ws/isaacsim/assets/vx300s_urdf.usd'
    create_vx300s_from_urdf(vx300s_urdf_path, vx300s_usd_path)
    print("Done")
    logger.info("Done")
    simulation_app.close()
