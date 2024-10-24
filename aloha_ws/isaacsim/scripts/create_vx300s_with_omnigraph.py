# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html
# launch Isaac Sim before any other imports
# default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import logging

import omni.graph.core as og
import omni.kit.app
import omni.usd

# Ref: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/logging.html
# `print(...)` outputs to Script Editor, while `logger.info(...)` outputs to console.
logger = logging.getLogger(__name__)


def create_vx300s_with_omnigraph():
    # Create vx300s
    vx300s_prim_path = "/World/vx300s_urdf"
    vx300s_joint_states_topic = "/vx300s/joint_states"
    vx300s_joint_command_topic = "/vx300s/joint_commands"
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
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("ConstantTokenTarget", "omni.graph.nodes.ConstantToken"),
                ("ToTarget", "omni.graph.nodes.ToTarget"),
                ("ConstantStringJointStatesTopic", "omni.graph.nodes.ConstantString"),
                ("ConstantStringJointCommandTopic", "omni.graph.nodes.ConstantString"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),

                # Ref: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ros2_bridge/docs/index.html#ros2context
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),

                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/GeneratedNodeDocumentation/OgnConstantToken.html
                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/GeneratedNodeDocumentation/OgnToTarget.html
                ("ConstantTokenTarget.inputs:value", "ToTarget.inputs:value"),
                ("ToTarget.outputs:converted", "PublishJointState.inputs:targetPrim"),
                ("ToTarget.outputs:converted", "ArticulationController.inputs:targetPrim"),

                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/GeneratedNodeDocumentation/OgnConstantString.html
                ("ConstantStringJointStatesTopic.inputs:value", "PublishJointState.inputs:topicName"),
                ("ConstantStringJointCommandTopic.inputs:value", "SubscribeJointState.inputs:topicName"),

                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Please refer to the `create_vx300s_from_urdf.py` file for reasons on using the root prim instead of root joint.
                ("ConstantTokenTarget.inputs:value", vx300s_prim_path),
                ("ConstantStringJointStatesTopic.inputs:value", vx300s_joint_states_topic),
                ("ConstantStringJointCommandTopic.inputs:value", vx300s_joint_command_topic),
            ],
        },
    )

if __name__ == '__main__':
    # Ref: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/extensions/enable-kit-extension.html
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
    print("ros2_bridge enabled: " + str(manager.is_extension_enabled("omni.isaac.ros2_bridge")))
    logger.info("ros2_bridge enabled: " + str(manager.is_extension_enabled("omni.isaac.ros2_bridge")))
    create_vx300s_with_omnigraph()
    vx300s_og_usd_path = '/home/ros2-essentials/aloha_ws/isaacsim/assets/vx300s_og.usd'
    omni.usd.get_context().save_as_stage(vx300s_og_usd_path)
    print("Done")
    logger.info("Done")
    simulation_app.close()
