# By setting `debug` as True, this script can be executed in the `Script Editor` in Isaac Sim GUI
debug = False
if not debug:
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html
    # launch Isaac Sim before any other imports
    # default first two lines in any standalone application
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})

import logging

import carb
import omni.graph.core as og
import omni.kit.app
import omni.usd
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.physics_context import PhysicsContext
from omni.isaac.core.utils.carb import set_carb_setting
from omni.isaac.core.utils.viewports import set_camera_view
from pxr import Usd, UsdGeom

# Ref: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/logging.html
# `print(...)` outputs to Script Editor, while `logger.info(...)` outputs to console.
logger = logging.getLogger(__name__)


def create_turtlebot3_burger_with_omnigraph():
    # Create turtlebot3_burger
    turtlebot3_burger_prim_path = "/World/turtlebot3_burger_urdf"
    camera_prim_path = "/World/turtlebot3_burger_urdf/base_scan/camera"
    camera_frame_id = "camera" # same as USD camera prim name
    lidar_prim_path = "/World/turtlebot3_burger_urdf/base_scan/lidar"
    lidar_frame_id = "lidar" # same as USD lidar prim name
    cmd_vel_topic = "/cmd_vel"
    joint_name_0 = "wheel_left_joint"
    joint_name_1 = "wheel_right_joint"
    stage: Usd.Stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(turtlebot3_burger_prim_path)
    prim.GetReferences().AddReference("/home/ros2-essentials/turtlebot3_ws/isaacsim/assets/turtlebot3_burger_urdf.usd")

    # Create Physics Scene and Ground Plane
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_interactive_scripting.html#isaac-sim-short-core-apis
    PhysicsContext()

    # Create Empty Environment (Ground Plane) or Existing Environment
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/assets/usd_assets_environments.html
    # GroundPlane("/World/GroundPlane")
    assets_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/"
    env_prim_path = "/World/environment"
    prim = stage.DefinePrim(env_prim_path)
    prim.GetReferences().AddReference(f"{assets_path}/Isaac/Environments/Office/office.usd")
    # Add ambient light for office environment
    carb_settings = carb.settings.get_settings()
    set_carb_setting(carb_settings, "/rtx/sceneDb/ambientLightIntensity", 1.0)
    # Set camera position
    camera_target = UsdGeom.Xformable(stage.GetPrimAtPath("/World/turtlebot3_burger_urdf")).GetLocalTransformation().ExtractTranslation()
    # The perspective view camera `/OmniverseKit_Persp` seems to be a special prim, whose attributes cannot be set
    # directly with normal APIs such as `Xformable.AddTranslateOp().Set`, `XformCommonAPI.SetTranslate`, and
    # `GetAttribute("xformOp:translate").Set`.
    set_camera_view(eye=camera_target + [0, 5, 3], target=camera_target, camera_prim_path="/OmniverseKit_Persp")

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
    # You can also hover your cursor on the OmniGraph node title to see its namespace and node name through the Isaac Sim GUI.

    # OmniGraph Scripting Notes:
    # Assuming you already have an working OmniGraph in Isaac Sim GUI,
    # - First, define the nodes from left to right, top to bottom.
    # - Second, define the outgoing edges based on the node order.
    # - Third, set the values of the input nodes.
    # Note that you can first type some pseudocode to plan the nodes and edges, and then look up the actual node name and input/output names.

    # Create OmniGraph
    # Refs:
    # - https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_omnigraph_scripting.html
    # - https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/howto/Controller.html
    # OmniGraph for Articulation Controller
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html#building-the-graph
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_DifferentialController", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("ScaleToFromStageUnit", "isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit"),
                ("Break3VectorAngularVel", "omni.graph.nodes.BreakVector3"),
                ("Break3VectorLinearVel", "omni.graph.nodes.BreakVector3"),
                ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
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

                # Ref: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/ogn/OgnIsaacArticulationController.html
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:targetPrim", turtlebot3_burger_prim_path),
                ("SubscribeTwist.inputs:topicName", cmd_vel_topic),
                # These three properties are hardcoded here, maybe it's better to infer them from the URDF/USD file.
                # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html#graph-explained
                ("DifferentialController.inputs:maxLinearSpeed", 0.22),
                ("DifferentialController.inputs:wheelDistance", 0.16),
                ("DifferentialController.inputs:wheelRadius", 0.025),
                ("ArticulationController.inputs:jointNames",
                    [
                        joint_name_0,
                        joint_name_1,
                    ],
                ),
            ],
        },
    )
    # OmniGraph for Publishing Clock
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_clock.html
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_Clock", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
            ],
        },
    )
    # OmniGraph for Publishing TF
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_tf.html
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_TF", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF.inputs:targetPrims", [
                    turtlebot3_burger_prim_path,
                    camera_prim_path,
                    lidar_prim_path,
                ]),
                ("PublishTF.inputs:topicName", "tf"),
            ]
        },
    )
    # OmniGraph for Publishing Joint State
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_manipulation.html
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_JointState", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:targetPrim", turtlebot3_burger_prim_path),
            ],
        },
    )
    # OmniGraph for Publishing Camera RGB/Depth/DepthPCL/Info
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_camera.html
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_Camera", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishCameraRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraDepthPCL", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                # PublishCameraRGB
                ("RenderProduct.outputs:execOut", "PublishCameraRGB.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishCameraRGB.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishCameraRGB.inputs:context"),
                # PublishCameraDepth
                ("RenderProduct.outputs:execOut", "PublishCameraDepth.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishCameraDepth.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishCameraDepth.inputs:context"),
                # PublishCameraDepthPCL
                ("RenderProduct.outputs:execOut", "PublishCameraDepthPCL.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishCameraDepthPCL.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishCameraDepthPCL.inputs:context"),
                # PublishCameraInfo
                ("RenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishCameraInfo.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", camera_prim_path),
                # PublishCameraRGB
                ("PublishCameraRGB.inputs:type", "rgb"),
                ("PublishCameraRGB.inputs:topicName", "rgb"),
                ("PublishCameraRGB.inputs:frameId", camera_frame_id),
                # PublishCameraDepth
                ("PublishCameraDepth.inputs:type", "depth"),
                ("PublishCameraDepth.inputs:topicName", "depth"),
                ("PublishCameraDepth.inputs:frameId", camera_frame_id),
                # PublishCameraDepthPCL
                ("PublishCameraDepthPCL.inputs:type", "depth_pcl"),
                ("PublishCameraDepthPCL.inputs:topicName", "depth_pcl"),
                ("PublishCameraDepthPCL.inputs:frameId", camera_frame_id),
                # PublishCameraInfo
                ("PublishCameraInfo.inputs:topicName", "camera_info"),
                ("PublishCameraInfo.inputs:topicNameRight", "camera_info_right"),
                ("PublishCameraInfo.inputs:frameId", camera_frame_id),
                ("PublishCameraInfo.inputs:frameIdRight", f"{camera_frame_id}_right"),
            ],
        },
    )
    # OmniGraph for Publishing (RTX) Lidar Laser Scan and Point Cloud
    # Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_rtx_lidar.html
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_LidarRTX", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishLidarLaserScan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("PublishLidarPointCloud", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                # PublishLidarLaserScan
                ("RenderProduct.outputs:execOut", "PublishLidarLaserScan.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishLidarLaserScan.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishLidarLaserScan.inputs:context"),
                # PublishLidarPointCloud
                ("RenderProduct.outputs:execOut", "PublishLidarPointCloud.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishLidarPointCloud.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishLidarPointCloud.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", lidar_prim_path),
                # PublishLidarLaserScan
                ("PublishLidarLaserScan.inputs:type", "laser_scan"),
                ("PublishLidarLaserScan.inputs:topicName", "laser_scan"),
                ("PublishLidarLaserScan.inputs:frameId", lidar_frame_id),
                # PublishLidarPointCloud
                ("PublishLidarPointCloud.inputs:type", "point_cloud"),
                ("PublishLidarPointCloud.inputs:topicName", "point_cloud"),
                ("PublishLidarPointCloud.inputs:frameId", lidar_frame_id),
                ("PublishLidarPointCloud.inputs:fullScan", True),
            ],
        },
    )

if __name__ == '__main__':
    # Ref: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/extensions/enable-kit-extension.html
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    print("ros2_bridge enabled: " + str(manager.is_extension_enabled("isaacsim.ros2.bridge")))
    logger.info("ros2_bridge enabled: " + str(manager.is_extension_enabled("isaacsim.ros2.bridge")))
    create_turtlebot3_burger_with_omnigraph()
    turtlebot3_burger_og_usd_path = '/home/ros2-essentials/turtlebot3_ws/isaacsim/assets/turtlebot3_burger_og.usd'
    omni.usd.get_context().save_as_stage(turtlebot3_burger_og_usd_path)
    print("Done")
    logger.info("Done")
    if not debug:
        simulation_app.close()
