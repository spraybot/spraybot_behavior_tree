/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */
#include <string>

#include "spraybot_behavior_tree/plugins/publish_string.hpp"

namespace spraybot_behavior_tree
{

PublishString::PublishString(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  getInput("topic_name", topic_name_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  rclcpp::PublisherOptions pub_option;
  string_pub_ = node_->create_publisher<std_msgs::msg::String>(
    topic_name_,
    rclcpp::SystemDefaultsQoS(), pub_option);

  RCLCPP_DEBUG(
    node_->get_logger(), "Initialized an PublishString BT node publishing to the topic %s",
    topic_name_.c_str());
}

PublishString::~PublishString()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down PublishString BT node");
}

BT::NodeStatus PublishString::tick()
{
  std_msgs::msg::String msg;
  getInput("string_value", msg.data);
  string_pub_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::PublishString>("PublishString");
}
