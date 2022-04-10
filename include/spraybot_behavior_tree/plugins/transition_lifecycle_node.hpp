/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <nav2_behavior_tree/bt_service_node.hpp>

namespace spraybot_behavior_tree
{

/**
 * @brief A spraybot_behavior_tree::BtServiceNode class that
 * wraps lifecyle_msgs::srv::ChangeState
 */
class TransitionLifecycleNode
  : public nav2_behavior_tree::BtServiceNode<lifecycle_msgs::srv::ChangeState>
{
public:
  /**
   * @brief A constructor for spraybot_behavior_tree::ConvertLL2Goal
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  TransitionLifecycleNode(
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
        BT::InputPort<std::string>(
          "transition_label",
          "Transition name: create/configure/activate/deactivate/cleanup"),
      });
  }

private:
  static std::unordered_map<std::string, unsigned int> transition_map;
};

}  // namespace spraybot_behavior_tree
