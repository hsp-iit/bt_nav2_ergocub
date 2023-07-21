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

#ifndef BT_NAV2_ERGOCUB__DECORATOR_ON_BOOL
#define BT_NAV2_ERGOCUB__DECORATOR_ON_BOOL

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace bt_nav2_ergocub
{

   /**
    * @brief A BT::DecoratorNode that ticks its child every time the robot
    * travels a specified distance
    */
   class DecoratorOnBool : public BT::DecoratorNode
   {
  public:
      DecoratorOnBool(const std::string& name, const BT::NodeConfiguration& conf);
    static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("service_name")}; }


      BT::NodeStatus tick() override;

      private:

      bool attachServer();


      std::string service_name_;
      rclcpp::Node::SharedPtr node_;
      bool is_first_time_, condition_changed_, server_attached_;
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_;
   };

}  // namespace 

#endif  // BT_NAV2_ERGOCUB__DECORATOR_ON_BOOL