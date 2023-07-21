// Copyright (c) 2021 Joshua Wallace
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

#include "bt_nav2_ergocub/is_path_valid.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace bt_nav2_ergocub
{

IsPathValidConditionModded::IsPathValidConditionModded(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

  path_valid_pub_ = node_ -> create_publisher<std_msgs::msg::Bool>("/is_path_valid", 10);
}

BT::NodeStatus IsPathValidConditionModded::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  auto result = client_->async_send_request(request);

  std_msgs::msg::Bool msg;
  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->is_valid) {
        msg.data = true;
        path_valid_pub_->publish(msg);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
        msg.data = false;
        path_valid_pub_->publish(msg);
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace ergocub_nav2_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ergocub_nav2_nodes::IsPathValidConditionModded>("IsPathValidConditionModded");
}