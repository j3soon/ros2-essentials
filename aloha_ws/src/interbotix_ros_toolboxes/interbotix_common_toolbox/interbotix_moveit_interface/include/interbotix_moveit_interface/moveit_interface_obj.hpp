// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERBOTIX_MOVEIT_INTERFACE__MOVEIT_INTERFACE_OBJ_HPP_
#define INTERBOTIX_MOVEIT_INTERFACE__MOVEIT_INTERFACE_OBJ_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_srvs/srv/empty.hpp"

#include "interbotix_moveit_interface_msgs/srv/move_it_plan.hpp"

namespace interbotix
{

using MoveItErrorCode = moveit::core::MoveItErrorCode;
using MoveItPlan = interbotix_moveit_interface_msgs::srv::MoveItPlan;
using Empty = std_srvs::srv::Empty;

// Constant defining the name of the planning group
inline static const std::string PLANNING_GROUP = "interbotix_arm";
inline static const std::string EE_LINK = "ee_gripper_link";
inline static const std::string VT_FRAME_NAME = "ee_pose";


class InterbotixMoveItInterface
{
public:
  /// @brief Constructor for the InterbotixMoveItInterface
  /// @param node SharedPtr to ROS2 node
  explicit InterbotixMoveItInterface(rclcpp::Node::SharedPtr & node);

  /// @brief Destructor for the InterbotixMoveItInterface
  ~InterbotixMoveItInterface();

  void init();

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified joint positions
  ///   [rad]
  /// @param joint_group_positions vector of joint positions [rad] to write to the joints;
  ///   sequence of positions match the joint name order expected by the 'interbotix_arm' group
  bool moveit_plan_joint_positions(const std::vector<double> joint_group_positions);

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector pose
  /// @param pose desired pose of the end-effector (frame is placed at the 'ee_arm_link') w.r.t.
  ///   the 'world' frame
  bool moveit_plan_ee_pose(const geometry_msgs::msg::Pose pose);

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector
  ///   position
  /// @param x translation along the 'X-axis' w.r.t. the 'world' frame
  /// @param y translation along the 'Y-axis' w.r.t. the 'world' frame
  /// @param z translation along the 'Z-axis' w.r.t. the 'world' frame
  bool moveit_plan_ee_position(double x, double y, double z);

  /// @brief Use MoveIt's planner to plan a trajectory to achieve the specified end-effector
  ///   orientation
  /// @param quat desired end-effector orientation expressed as a quaternion
  bool moveit_plan_ee_orientation(const geometry_msgs::msg::Quaternion quat);

  /// @brief Use MoveIt's planner to plan a trajectory to move the end-effector to the specified
  ///   waypoints
  /// @param waypoints sequence of goal poses for the end-effector to achieve; goal poses are
  ///   relative to the 'world' frame
  bool moveit_plan_cartesian_path(const std::vector<geometry_msgs::msg::Pose> waypoints);

  /// @brief Execute a Moveit plan on the robot arm
  bool moveit_execute_plan(void);

  /// @brief Set path constraints for a specific link
  /// @param constrained_link name of the link to constrain as defined in the URDF
  /// @param reference_link name of the link that the path of the 'constrained_link' is being
  ///   constrained against
  /// @param quat desired orientation of the 'constrained_link' relative to the 'reference_link'
  /// @param tolerance allowable deviation [rad] from the constraint about the x, y, and z axes
  void moveit_set_path_constraint(
    const std::string constrained_link,
    const std::string reference_link,
    const geometry_msgs::msg::Quaternion quat,
    const double tolerance);

  /// @brief Remove any path constraints
  void moveit_clear_path_constraints(void);

  /// @brief Return the current end-effector pose relative to the 'world' frame
  geometry_msgs::msg::Pose moveit_get_ee_pose(void);

  /// @brief Scale the end-effector velocity down from the max velocity specified in the robot
  ///   model
  /// @param factor a double between 0 and 1.
  void moveit_scale_ee_velocity(const double factor);

private:
  // ROS Node
  rclcpp::Node::SharedPtr node_;

  // Service to plan or execute a goal pose for the end-effector
  rclcpp::Service<MoveItPlan>::SharedPtr srv_moveit_plan;

  // Service to clear the MoveItVisualTools markers
  rclcpp::Service<Empty>::SharedPtr srv_clear_markers;

  // Pose of text w.r.t. the 'world' frame in RViz
  Eigen::Isometry3d text_pose;

  // Holds the joints in the 'interbotix_arm' group
  const moveit::core::JointModelGroup * joint_model_group;

  // Used to display text and other markers in RViz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;

  // MoveIt object that can actually plan and execute trajectories
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

  // Plan object that holds the calculated trajectory
  moveit::planning_interface::MoveGroupInterface::Plan saved_plan;

  // Not applied in this demo but would be used to add objects to the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /// @brief ROS Service callback to plan or execute a desired end-effector pose, position, or
  ///   orientation
  /// @param req custom message of type 'MoveItPlan'. Look at the service message for details
  /// @param res [out] a boolean specifying whether the plan or execution was successful and a
  ///   'string' message saying likewise
  bool moveit_planner(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<MoveItPlan::Request> req,
    std::shared_ptr<MoveItPlan::Response> res);

  /// @brief ROS Service to clear MoveItVisualTools markers
  /// @param req the Empty request, unused
  /// @param res [out] the empty response, unused
  bool clear_markers(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<Empty::Request> req,
    std::shared_ptr<Empty::Response> res);
};

}  // namespace interbotix

#endif  // INTERBOTIX_MOVEIT_INTERFACE__MOVEIT_INTERFACE_OBJ_HPP_
