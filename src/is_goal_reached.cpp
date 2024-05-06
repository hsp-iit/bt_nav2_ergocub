// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "bt_nav2_ergocub/is_goal_reached.hpp"

#include "yarp/os/Network.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::literals::chrono_literals;

namespace ergocub_nav2_nodes
{

GoalReachedConditionModded::GoalReachedConditionModded(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  publish_info_ = false;
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
}

GoalReachedConditionModded::~GoalReachedConditionModded()
{
  //yarp.fini();
  cleanup();
}

void GoalReachedConditionModded::initialize()
{
  std::cout << "[GoalReachedConditionModded] init..." << std::endl;
  //yarp.init();
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  nav2_util::declare_parameter_if_not_declared(
    node_, "nav_status_port_name",
    rclcpp::ParameterValue("/is_goal_reached_bt/goal_reached:o"));
  node_->get_parameter<std::string>("nav_status_port_name", nav_status_port_name_);
  if (nav_status_port_name_=="")
  {
    nav_status_port_name_ = "/is_goal_reached_bt/goal_reached:o";
  }
  std::cout << "[GoalReachedConditionModded] nav_status_port_name: " << nav_status_port_name_ << std::endl;

  nav2_util::declare_parameter_if_not_declared(
    node_, "perception_bt_port_name",
    rclcpp::ParameterValue("/BT/RobotNavigating"));
  node_->get_parameter<std::string>("perception_bt_port_name", perception_bt_port_name_);
  if (perception_bt_port_name_=="")
  {
    perception_bt_port_name_ = "/BT/RobotNavigating";
  }
  std::cout << "[GoalReachedConditionModded] nav_status_port_name: " << perception_bt_port_name_ << std::endl;

  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_angular_tol",
    rclcpp::ParameterValue(0.14));
  node_->get_parameter<double>("goal_angular_tol", goal_angular_tol_);
  if (goal_angular_tol_ < 0.1)
  {
    goal_angular_tol_=0.1;
  }
  std::cout << "[GoalReachedConditionModded] goal_angular_tol: " << goal_angular_tol_ << std::endl;

  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_reached_tol",
    rclcpp::ParameterValue(0.3));
  node_->get_parameter<double>("goal_reached_tol", goal_reached_tol_);
  if (goal_reached_tol_ < 0.05)
  {
    goal_angular_tol_=0.05;
  }
  std::cout << "[GoalReachedConditionModded] goal_reached_tol: " << goal_reached_tol_ << std::endl;

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  yarp_port_.open(nav_status_port_name_);
  // YARP connection check
  yarp::os::Network::connect(nav_status_port_name_, perception_bt_port_name_);
  if(yarp::os::Network::isConnected(nav_status_port_name_, perception_bt_port_name_))
  {
      RCLCPP_INFO(node_->get_logger(), "YARP Ports connected successfully");
      publish_info_ = true;
  } 
  else 
  {
    publish_info_=false;
  }

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  goal_state_pub_=node_->create_publisher<std_msgs::msg::Bool>("/is_goal_reached", 10);

  initialized_ = true;
}

BT::NodeStatus GoalReachedConditionModded::tick()
{
  if (!initialized_) {
    initialize();
  }
  std_msgs::msg::Bool msg;
  if (isGoalReached()) {
    msg.data = true;
    goal_state_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "[GoalReachedConditionModded] Goal Reached!");
    return BT::NodeStatus::SUCCESS;
  }
    msg.data = false;
    goal_state_pub_->publish(msg);
  return BT::NodeStatus::FAILURE;
}

bool GoalReachedConditionModded::isGoalReached()
{
  geometry_msgs::msg::PoseStamped current_pose;

  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;
  //orientation handling
  tf2::Quaternion q_goal;
  tf2::fromMsg(goal.pose.orientation, q_goal);
  tf2::Quaternion q_pose;
  tf2::fromMsg(current_pose.pose.orientation, q_pose);
  tf2::Matrix3x3 m_g(q_goal);
  double r_g, p_g, yaw_g;
  m_g.getRPY(r_g, p_g, yaw_g);
  tf2::Matrix3x3 m_p(q_pose);
  double r_p, p_p, yaw_p;
  m_p.getRPY(r_p, p_p, yaw_p);

  tf2::Matrix3x3 m(q_goal*q_pose.inverse());
  double r, p, yaw;
  m.getRPY(r, p, yaw);
  std::cout << "[GoalReachedConditionModded] relative yaw difference rad: " << yaw << " degrees: " << yaw * 180 / M_PI << std::endl;
  std::cout << "[GoalReachedConditionModded] pose yaw rad: " << yaw_p << " degrees: " << yaw_p * 180 / M_PI << std::endl;
  std::cout << "[GoalReachedConditionModded] Goal X: " << goal.pose.position.x << " Y: " << goal.pose.position.y << " Yaw: " << yaw_g << std::endl;
  std::cout << "[GoalReachedConditionModded] Current Pose X: " << current_pose.pose.position.x << " Y: " << current_pose.pose.position.y << std::endl;
  std::cout << "[GoalReachedConditionModded] Distance: " << std::sqrt(dx * dx + dy * dy) << std::endl;
  std::cout << "[GoalReachedConditionModded] Global Frame: " << global_frame_ << " Robot Frame: " << robot_base_frame_ << std::endl;

  //if we have to check for angular alignment
  if (check_angular_alignment_)
  {
    if ((dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_) && (std::abs(yaw) <= goal_angular_tol_))
    {
      if (publish_info_)
      {
        RCLCPP_INFO(node_->get_logger(), "[GoalReachedConditionModded] Publishing Info!");
        auto& out = yarp_port_.prepare();
        out.clear();
        out.addInt32(1);  //Goal NOT Reached
        //yarp_port_.write();
      }
      return true;
    }
    else
    {
      if (publish_info_)
      {
        RCLCPP_INFO(node_->get_logger(), "[GoalReachedConditionModded] Publishing Info!");
        auto& out = yarp_port_.prepare();
        out.clear();
        out.addInt32(0);  //Goal Reached
        //yarp_port_.write();
      }
      return false;
    }
  }
  else  // DON'T check angular alignment with goal
  {
    if ((dx * dx + dy * dy) <= (goal_reached_tol_ * goal_reached_tol_))
    {
      if (publish_info_)
      {
        RCLCPP_INFO(node_->get_logger(), "[GoalReachedConditionModded] Publishing Info!");
        auto& out = yarp_port_.prepare();
        out.clear();
        out.addInt32(1);  //Goal NOT Reached
        //yarp_port_.write();
      }
      return true;
    }
    else
    {
      if (publish_info_)
      {
        RCLCPP_INFO(node_->get_logger(), "[GoalReachedConditionModded] Publishing Info!");
        auto& out = yarp_port_.prepare();
        out.clear();
        out.addInt32(0);  //Goal Reached
        //yarp_port_.write();
      }
      return false;
    }
  }
}

}  // namespace ergocub_nav2_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ergocub_nav2_nodes::GoalReachedConditionModded>("GoalReachedConditionModded");
}
