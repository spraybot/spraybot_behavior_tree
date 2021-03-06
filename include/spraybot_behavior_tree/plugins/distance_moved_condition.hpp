// Copyright (c) 2020 Sarthak Mittal
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

#pragma once

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace spraybot_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS every time the robot
 * travels a specified distance and FAILURE otherwise
 */
class DistanceMoved : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for spraybot_behavior_tree::DistanceMoved
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  DistanceMoved(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  DistanceMoved() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 1.0, "Distance"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  geometry_msgs::msg::PoseStamped start_pose_;

  double distance_;
  double transform_tolerance_;
  bool is_computing_distance;
  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree
