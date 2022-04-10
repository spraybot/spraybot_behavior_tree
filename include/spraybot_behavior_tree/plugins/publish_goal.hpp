/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp_v3/action_node.h>

namespace spraybot_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to get publish a goal
 */
class PublishGoal : public BT::ActionNodeBase
{
public:
  /**
   * @brief A spraybot_behavior_tree::NextGoal constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PublishGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~PublishGoal();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<std::vector<double>>("goal_pose", "Goal pose vector (x; y; yaw)"),
        BT::InputPort <std::string> ("frame", "map", "Frame name to give goal in")
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

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

}  // namespace spraybot_behavior_tree
