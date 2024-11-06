# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html
# launch Isaac Sim before any other imports
# default first two lines in any standalone application
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import logging

import omni.graph.core as og
import omni.kit.app
import omni.usd

from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.physics_context import PhysicsContext

# Ref: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/logging.html
# `print(...)` outputs to Script Editor, while `logger.info(...)` outputs to console.
logger = logging.getLogger(__name__)


def create_turtlebot3_burger_with_omnigraph():
    # Create turtlebot3_burger
    turtlebot3_burger_prim_path = "/World/turtlebot3_burger_urdf"
    cmd_vel_topic = "/cmd_vel"
    joint_name_0 = "wheel_left_joint"
    joint_name_1 = "wheel_right_joint"
    stage = omni.usd.get_context().get_stage()
    stage.DefinePrim(turtlebot3_burger_prim_path)
    prim = stage.GetPrimAtPath(turtlebot3_burger_prim_path)
    prim.GetReferences().AddReference("/home/ros2-essentials/turtlebot3_ws/isaacsim/assets/turtlebot3_burger_urdf.usd")

    # Create Physics Scene and Ground Plane
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_interactive_scripting.html#isaac-sim-short-core-apis
    PhysicsContext()
    GroundPlane("/World/GroundPlane")

    # OmniGraph Nodes References:
    # - OmniGraph Node Library
    #   https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/Overview.html
    #   https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph/node-library/node-library.html
    # - omni.graph.action_nodes
    #   https://docs.omniverse.nvidia.com/kit/docs/omni.graph.action_nodes/latest/Overview.html
    # - omni.isaac.core_nodes
    #   https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/index.html
    # - omni.isaac.ros2_bridge
    #   https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ros2_bridge/docs/index.html
    # You can also hover your cursor on the OmniGraph node title to see its namespace and node name.

    # OmniGraph Scripting Notes:
    # Assuming you already have an working OmniGraph in Isaac Sim GUI,
    # - First, define the nodes from left to right, top to bottom.
    # - Second, define the outgoing edges based on the node order.
    # - Third, set the values of the input nodes.
    # Note that you can first type some pseudocode to plan the nodes and edges, and then look up the actual node name and input/output names.

    # Create OmniGraph
    # Refs:
    # - https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_manipulation.html#add-joint-states-in-extension
    # - https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_omnigraph_scripting.html
    # - https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html#building-the-graph
    # - https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/howto/Controller.html
    # TODO: Consider adding more OmniGraphs for publishing JointState and Clock
    # TODO: Simplify the OmniGraph by removing unnecessary constant nodes
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("ScaleToFromStageUnit", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                ("Break3VectorAngularVel", "omni.graph.nodes.BreakVector3"),
                ("Break3VectorLinearVel", "omni.graph.nodes.BreakVector3"),
                ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
                ("MakeArray", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CREATE_ATTRIBUTES: [
                ("MakeArray.inputs:input1", "token"),
            ],
            og.Controller.Keys.CONNECT: [
                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.action_nodes/latest/GeneratedNodeDocumentation/OgnOnPlaybackTick.html
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

                # Ref: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ros2_bridge/docs/ogn/OgnROS2Context.html
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),

                # Ref: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ros2_bridge/docs/ogn/OgnROS2SubscribeTwist.html
                ("SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
                ("SubscribeTwist.outputs:angularVelocity", "Break3VectorAngularVel.inputs:tuple"),
                ("SubscribeTwist.outputs:linearVelocity", "ScaleToFromStageUnit.inputs:value"),

                # Ref: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/ogn/OgnIsaacScaleToFromStageUnit.html
                ("ScaleToFromStageUnit.outputs:result", "Break3VectorLinearVel.inputs:tuple"),
                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/GeneratedNodeDocumentation/OgnBreakVector3.html
                ("Break3VectorAngularVel.outputs:z", "DifferentialController.inputs:angularVelocity"),
                ("Break3VectorLinearVel.outputs:x", "DifferentialController.inputs:linearVelocity"),

                # Ref: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.wheeled_robots/docs/ogn/OgnDifferentialController.html
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),

                # Ref: https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/GeneratedNodeDocumentation/OgnConstructArray.html
                ("MakeArray.outputs:array", "ArticulationController.inputs:jointNames"),

                # Ref: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/ogn/OgnIsaacArticulationController.html
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:targetPrim", turtlebot3_burger_prim_path),
                ("MakeArray.inputs:arrayType", "token[]"),
                ("MakeArray.inputs:input0", joint_name_0),
                ("MakeArray.inputs:input1", joint_name_1),
                ("MakeArray.inputs:arraySize", 2),
                ("SubscribeTwist.inputs:topicName", cmd_vel_topic),
                # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html#graph-explained
                ("DifferentialController.inputs:maxLinearSpeed", 0.22),
                ("DifferentialController.inputs:wheelDistance", 0.16),
                ("DifferentialController.inputs:wheelRadius", 0.025),
            ],
        },
    )

if __name__ == '__main__':
    # Ref: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/extensions/enable-kit-extension.html
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
    print("ros2_bridge enabled: " + str(manager.is_extension_enabled("omni.isaac.ros2_bridge")))
    logger.info("ros2_bridge enabled: " + str(manager.is_extension_enabled("omni.isaac.ros2_bridge")))
    create_turtlebot3_burger_with_omnigraph()
    turtlebot3_burger_og_usd_path = '/home/ros2-essentials/turtlebot3_ws/isaacsim/assets/turtlebot3_burger_og.usd'
    omni.usd.get_context().save_as_stage(turtlebot3_burger_og_usd_path)
    print("Done")
    logger.info("Done")
    simulation_app.close()
