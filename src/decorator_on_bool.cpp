/*
 *   Copyright (c) 2022 Michele Colledanchise
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/decorator_node.h>

#include <bt_nav2_ergocub/decorator_on_bool.hpp>

using namespace std::literals::chrono_literals;  // for "s" literals 

namespace bt_nav2_ergocub
{

   DecoratorOnBool::DecoratorOnBool(const std::string& name, const BT::NodeConfiguration& conf)
       : BT::DecoratorNode(name, conf),
         condition_changed_(true)  // to trigger it the first time I set condition changed to true
   {
      
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      getInput("service_name", service_name_); 
      server_attached_ = attachServer();
      RCLCPP_INFO(node_->get_logger(),
                     "starting decorator ...");
   }

   bool DecoratorOnBool::attachServer()
   {
      service_client_ = node_->create_client<std_srvs::srv::Trigger>(service_name_.c_str());

      while (!service_client_->wait_for_service())
      {
         if (!rclcpp::ok())
         {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
         }
         RCLCPP_INFO(node_->get_logger(),
                     "service not available, waiting again...");
      }
      return true;
   }

   inline BT::NodeStatus DecoratorOnBool::tick()
   {
      if (!server_attached_)
      {
         RCLCPP_ERROR(node_->get_logger(),
                      "The server is not attached to the BT node. ");
         return BT::NodeStatus::FAILURE;
      }

      setStatus(BT::NodeStatus::RUNNING);

      // check condition value

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = service_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {

      condition_changed_ = result.get()->success;
      
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s",
                   service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

      if (condition_changed_ || (child_node_->status() == BT::NodeStatus::RUNNING)) 
      {
         condition_changed_ = false;
         const BT::NodeStatus child_status = child_node_->executeTick();
         RCLCPP_INFO(node_->get_logger(),
                     "child_status ...");
         return child_status;
      }
      else
      {
      RCLCPP_INFO(node_->get_logger(),
                     "Skipping child  ...");
      	 return BT::NodeStatus::SUCCESS;
      }

      return status();
   }

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
   factory.registerNodeType<bt_nav2_ergocub::DecoratorOnBool>("DecoratorOnBool");
}