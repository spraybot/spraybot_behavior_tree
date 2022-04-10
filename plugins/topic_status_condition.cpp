/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */
#include <string>
#include <chrono>

#include "spraybot_behavior_tree/plugins/topic_status_condition.hpp"

namespace spraybot_behavior_tree
{

TopicStatusCondition::TopicStatusCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  status_(false)
{
  getInput("topic_name", topic_name_);

  bool initial_status;
  getInput("inital_status", initial_status);
  status_ = initial_status;

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_name_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TopicStatusCondition::onStatusReceived, this, std::placeholders::_1),
    sub_option);

  RCLCPP_DEBUG(
    node_->get_logger(), "Initialized an TopicStatusCondition BT node subscribing to the topic %s",
    topic_name_.c_str());
}

TopicStatusCondition::~TopicStatusCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down TopicStatusCondition BT node");
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

void TopicStatusCondition::onStatusReceived(const typename std_msgs::msg::Bool::SharedPtr msg)
{
  status_.store(msg->data);
}

BT::NodeStatus TopicStatusCondition::tick()
{
  if (status_.load()) {
    bool initial_status;
    getInput("inital_status", initial_status);
    status_.store(initial_status);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::TopicStatusCondition>("TopicStatus");
}
