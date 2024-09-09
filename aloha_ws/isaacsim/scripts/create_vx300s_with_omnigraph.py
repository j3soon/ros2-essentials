# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html
# launch Isaac Sim before any other imports
# default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import logging

import omni.graph.core as og
import omni.kit.app
import omni.usd
from omni.isaac.core import World

logger = logging.getLogger(__name__)


def log(msg):
    # Print to console
    # Ref: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/logging.html
    logger.info(msg)
    # Print to Script Editor
    print(msg)

def create_vx300s_with_omnigraph():
    # Create vx300s
    vx300s_prim_path = "/World/vx300s_urdf"
    stage = omni.usd.get_context().get_stage()
    stage.DefinePrim(vx300s_prim_path)
    prim = stage.GetPrimAtPath(vx300s_prim_path)
    prim.GetReferences().AddReference("/home/ros2-essentials/aloha_ws/isaacsim/assets/vx300s_urdf.usd")

    # Create OmniGraph
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_manipulation.html#add-joint-states-in-extension
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_omnigraph_scripting.html
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ConstantToken", "omni.graph.nodes.ConstantToken"),
                ("ToTarget", "omni.graph.nodes.ToTarget"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/GeneratedNodeDocumentation/OgnConstantToken.html
                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/GeneratedNodeDocumentation/OgnToTarget.html
                ("ConstantToken.inputs:value", "ToTarget.inputs:value"),
                ("ToTarget.outputs:converted", "PublishJointState.inputs:targetPrim"),
                ("ToTarget.outputs:converted", "ArticulationController.inputs:targetPrim"),

                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Please refer to the `create_vx300s_from_urdf.py` file for reasons on using the root prim instead of root joint.
                ("ConstantToken.inputs:value", vx300s_prim_path),
            ],
        },
    )

if __name__ == '__main__':
    # Ref: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/extensions/enable-kit-extension.html
    manager = omni.kit.app.get_app().get_extension_manager()
    # enable immediately
    manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
    log("ros2_bridge enabled: " + str(manager.is_extension_enabled("omni.isaac.ros2_bridge")))
    create_vx300s_with_omnigraph()
    vx300s_og_usd_path = '/home/ros2-essentials/aloha_ws/isaacsim/assets/vx300s_og.usd'
    omni.usd.get_context().save_as_stage(vx300s_og_usd_path)
    log("Done!")
    # # Ref: `~/.local/share/ov/pkg/isaac-sim-4.1.0/standalone_examples/api/omni.isaac.franka/follow_target_with_rmpflow.py`
    # my_world = World(stage_units_in_meters=1.0)
    # reset_needed = False
    # while simulation_app.is_running():
    #     my_world.step(render=True)
    #     if my_world.is_stopped() and not reset_needed:
    #         reset_needed = True
    #     if my_world.is_playing():
    #         if reset_needed:
    #             my_world.reset()
    #             reset_needed = False
    simulation_app.close()
