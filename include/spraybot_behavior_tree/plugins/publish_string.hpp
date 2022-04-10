/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <std_msgs/msg/string.hpp>

namespace spraybot_behavior_tree
{

/**
 * @brief A BT::ConditionNode that checks subscribes to a topic with status information and
 * returns SUCCESS or FAILURE based on the status
 */
class PublishString : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for spraybot_behavior_tree::PublishString
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PublishString(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  PublishString() = delete;

  /**
   * @brief A destructor for spraybot_behavior_tree::PublishString
   */
  ~PublishString() override;

  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

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
      BT::InputPort<std::string>("topic_name", std::string("/test_string"), "String topic name"),
      BT::InputPort<bool>("string_value", "test", "String value")
    };
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  std::string topic_name_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
};

}  // namespace spraybot_behavior_tree
