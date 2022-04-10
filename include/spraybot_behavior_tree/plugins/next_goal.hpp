/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp_v3/action_node.h>

namespace spraybot_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to get the first goal from a list of goals
 */
class NextGoal : public BT::ActionNodeBase
{
public:
  /**
   * @brief A spraybot_behavior_tree::NextGoal constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  NextGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
          "input_goals",
          "Destinations to plan through"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
          "output_goals",
          "Destinations to plan through, with current goal removed")
      };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;
};

}  // namespace spraybot_behavior_tree
