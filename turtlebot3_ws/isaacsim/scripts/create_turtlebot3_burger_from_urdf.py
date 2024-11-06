# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html
# launch Isaac Sim before any other imports
# default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import logging

import omni.kit.commands
import omni.usd
from omni.importer.urdf import _urdf
from pxr import UsdPhysics, PhysxSchema

# Ref: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/logging.html
# `print(...)` outputs to Script Editor, while `logger.info(...)` outputs to console.
logger = logging.getLogger(__name__)


def create_turtlebot3_burger_from_urdf(urdf_path, usd_path):
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html#importing-urdf-using-python
    # Set the settings in the import config
    import_config = _urdf.ImportConfig()
    import_config.fix_base = False
    import_config.make_default_prim = True
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
    # Finally import the robot & save it as USD
    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=urdf_path,
        import_config=import_config, dest_path=usd_path,
    )
    omni.usd.get_context().open_stage(usd_path)
    stage = omni.usd.get_context().get_stage()

    # Articulation Root for a floating-base articulation can be either:
    # (1) the root rigid-body link, or
    # (2) an ancestor of the root link in the USD hierarchy.
    # Ref: https://docs.omniverse.nvidia.com/extensions/latest/ext_physics/articulations.html#articulationroot
    # The URDF Importer uses the first option to define articulation root for the imported robot.
    # Using the first option works fine, but we'll follow the docs to switch to the second option to avoid potential future issues.
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html#graph-explained
    prim = stage.GetPrimAtPath("/turtlebot3_burger")
    articulation = UsdPhysics.ArticulationRootAPI.Apply(prim)
    physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(articulation.GetPrim())
    physx_articulation.CreateEnabledSelfCollisionsAttr().Set(False)
    prim = stage.GetPrimAtPath("/turtlebot3_burger/base_footprint")
    prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

    omni.usd.get_context().save_stage()

if __name__ == '__main__':
    turtlebot3_burger_urdf_path = f'/home/ros2-essentials/turtlebot3_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf'
    turtlebot3_burger_usd_path = '/home/ros2-essentials/turtlebot3_ws/isaacsim/assets/turtlebot3_burger_urdf.usd'
    create_turtlebot3_burger_from_urdf(turtlebot3_burger_urdf_path, turtlebot3_burger_usd_path)
    print("Done")
    logger.info("Done")
    simulation_app.close()
