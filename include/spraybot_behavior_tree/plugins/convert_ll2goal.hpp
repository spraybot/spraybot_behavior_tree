/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <nav2_behavior_tree/bt_service_node.hpp>

namespace spraybot_behavior_tree
{

/**
 * @brief A spraybot_behavior_tree::BtServiceNode class that
 * wraps robot_localization::srv::FromLL
 */
class ConvertLL2Goal
  : public nav2_behavior_tree::BtServiceNode<robot_localization::srv::FromLL>
{
public:
  /**
   * @brief A constructor for spraybot_behavior_tree::ConvertLL2Goal
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ConvertLL2Goal(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the service
   * @return BT::NodeStatus Status of completion of request
   */
  BT::NodeStatus on_completion() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "ll_goal",
          "Input goal in geographic coordinates"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "goal",
          "Output goal in local coordinates"),
      });
  }
};

}  // namespace spraybot_behavior_tree
