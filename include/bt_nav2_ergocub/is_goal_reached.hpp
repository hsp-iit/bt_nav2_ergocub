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

#ifndef ERGOCUB_NAV2_NODES__GOAL_REACHED_CONDITION_MODDED_HPP_
#define ERGOCUB_NAV2_NODES__GOAL_REACHED_CONDITION_MODDED_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"

#include "yarp/os/BufferedPort.h"
#include "yarp/os/Network.h"
#include "yarp/os/Bottle.h"

namespace ergocub_nav2_nodes
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class GoalReachedConditionModded : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalReachedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GoalReachedConditionModded(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GoalReachedConditionModded() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::GoalReachedCondition
   */
  ~GoalReachedConditionModded() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool true when goal is reached, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

protected:
  /**
   * @brief Cleanup function
   */
  void cleanup()
  {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool initialized_;
  double goal_reached_tol_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;

  std::string nav_status_port_name_;
  double goal_angular_tol_;

  yarp::os::Network yarp;
  yarp::os::BufferedPort<yarp::os::Bottle> yarp_port_;
  
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_state_pub_;
};

}  // namespace ergocub_nav2_nodes

#endif  // ERGOCUB_NAV2_NODES__GOAL_REACHED_CONDITION_MODDED_HPP_
