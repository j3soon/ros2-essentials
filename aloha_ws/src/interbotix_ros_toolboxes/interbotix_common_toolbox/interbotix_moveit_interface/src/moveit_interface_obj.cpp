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

#include "interbotix_moveit_interface/moveit_interface_obj.hpp"

#include <memory>
#include <string>
#include <vector>

namespace interbotix
{

InterbotixMoveItInterface::InterbotixMoveItInterface(
  rclcpp::Node::SharedPtr & node)
: node_(node)
{
  // Create executor to spin just in the constructor so MoveGroupInterface can initialize
  auto exec_temp = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_temp->add_node(node_);
  std::thread([&exec_temp]() {exec_temp->spin();}).detach();

  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_,
    PLANNING_GROUP);
  joint_model_group = move_group->getCurrentState(2.0)->getJointModelGroup(PLANNING_GROUP);

  srv_moveit_plan = node_->create_service<MoveItPlan>(
    "moveit_plan",
    std::bind(
      &interbotix::InterbotixMoveItInterface::moveit_planner,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  srv_clear_markers = node_->create_service<Empty>(
    "clear_markers",
    std::bind(
      &interbotix::InterbotixMoveItInterface::clear_markers,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
    node_,
    move_group->getPlanningFrame(),
    "/moveit_visual_tools");
  visual_tools->deleteAllMarkers();
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.770;
  visual_tools->publishText(
    text_pose,
    "InterbotixMoveItInterface",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->trigger();

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(
    node_->get_logger(),
    "Reference frame: %s", move_group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(
    node_->get_logger(),
    "End effector link: %s", move_group->getEndEffectorLink().c_str());

  // Stop executor and remove node from it so exec_ can be used later
  exec_temp->cancel();
  exec_temp->remove_node(node_);
}

InterbotixMoveItInterface::~InterbotixMoveItInterface()
{
  delete joint_model_group;
}

bool InterbotixMoveItInterface::moveit_plan_joint_positions(
  const std::vector<double> joint_group_positions)
{
  visual_tools->deleteAllMarkers();
  move_group->setJointValueTarget(joint_group_positions);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);

  visual_tools->publishText(
    text_pose,
    "Joint Space Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_pose(const geometry_msgs::msg::Pose pose)
{
  visual_tools->deleteAllMarkers();
  move_group->setPoseTarget(pose);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);

  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Pose Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan success: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_position(double x, double y, double z)
{
  visual_tools->deleteAllMarkers();
  move_group->setPositionTarget(x, y, z);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  RCLCPP_INFO(
    node_->get_logger(),
    "Target: x, y, z: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);

  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);

  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Position Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan success: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_orientation(
  const geometry_msgs::msg::Quaternion quat)
{
  visual_tools->deleteAllMarkers();
  move_group->setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);
  geometry_msgs::msg::Pose pose;
  pose = moveit_get_ee_pose();
  pose.orientation = quat;
  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Orientation Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan successful: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_cartesian_path(
  const std::vector<geometry_msgs::msg::Pose> waypoints)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group->computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory);
  RCLCPP_INFO(
    node_->get_logger(),
    "Visualizing (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  visual_tools->deleteAllMarkers();
  visual_tools->publishText(
    text_pose,
    "Cartesian Path",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishPath(
    waypoints,
    rviz_visual_tools::LIME_GREEN,
    rviz_visual_tools::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    visual_tools->publishAxisLabeled(
      waypoints[i],
      "pt" + std::to_string(i),
      rviz_visual_tools::SMALL);
  }
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  saved_plan = plan;
  saved_plan.trajectory_ = trajectory;

  // If a plan was found for over 90% of the waypoints...
  // consider that a successful planning attempt
  return (1.0 - fraction < 0.1) && (fraction != -1.0);
}

bool InterbotixMoveItInterface::moveit_execute_plan(void)
{
  return move_group->execute(saved_plan) == MoveItErrorCode::SUCCESS;
}

void InterbotixMoveItInterface::moveit_set_path_constraint(
  const std::string constrained_link,
  const std::string reference_link,
  const geometry_msgs::msg::Quaternion quat,
  const double tolerance)
{
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = constrained_link;
  ocm.header.frame_id = reference_link;
  ocm.orientation = quat;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;

  // this parameter sets the importance of this constraint relative to other constraints that might
  // be present. Closer to '0' means less important.
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group->setPathConstraints(test_constraints);

  // Since there is a constraint, it might take the planner a lot longer to come up with a valid
  // plan - so give it some time
  move_group->setPlanningTime(30.0);
}

void InterbotixMoveItInterface::moveit_clear_path_constraints(void)
{
  move_group->clearPathConstraints();

  // Now that there are no constraints, reduce the planning time to the default
  move_group->setPlanningTime(5.0);
}

geometry_msgs::msg::Pose InterbotixMoveItInterface::moveit_get_ee_pose(void)
{
  return move_group->getCurrentPose().pose;
}

void InterbotixMoveItInterface::moveit_scale_ee_velocity(const double factor)
{
  move_group->setMaxVelocityScalingFactor(factor);
}

bool InterbotixMoveItInterface::moveit_planner(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<MoveItPlan::Request> req,
  std::shared_ptr<MoveItPlan::Response> res)
{
  (void)request_header;
  bool success = false;
  std::string service_type;
  if (req->cmd == MoveItPlan::Request::CMD_PLAN_POSE) {
    success = moveit_plan_ee_pose(req->ee_pose);
    service_type = "Planning EE pose";
  } else if (req->cmd == MoveItPlan::Request::CMD_PLAN_POSITION) {
    success = moveit_plan_ee_position(
      req->ee_pose.position.x,
      req->ee_pose.position.y,
      req->ee_pose.position.z);
    service_type = "Planning EE position";
  } else if (req->cmd == MoveItPlan::Request::CMD_PLAN_ORIENTATION) {
    success = moveit_plan_ee_orientation(req->ee_pose.orientation);
    service_type = "Planning EE orientation";
  } else if (req->cmd == MoveItPlan::Request::CMD_EXECUTE) {
    success = moveit_execute_plan();
    service_type = "Execution";
  }
  res->success = success;
  if (success) {
    res->msg.data = service_type + " was successful!";
  } else {
    res->msg.data = service_type + " was not successful.";
  }

  return true;
}

bool InterbotixMoveItInterface::clear_markers(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<Empty::Request> req,
  std::shared_ptr<Empty::Response> res)
{
  (void)request_header;
  (void)req;
  (void)res;
  visual_tools->deleteAllMarkers();
  visual_tools->trigger();
  RCLCPP_DEBUG(node_->get_logger(), "Cleared markers.");
  return true;
}

}  // namespace interbotix
