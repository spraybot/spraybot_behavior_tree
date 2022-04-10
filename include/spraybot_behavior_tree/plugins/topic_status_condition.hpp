/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <string>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"

namespace spraybot_behavior_tree
{

/**
 * @brief A BT::ConditionNode that checks subscribes to a topic with status information and
 * returns SUCCESS or FAILURE based on the status
 */
class TopicStatusCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for spraybot_behavior_tree::TopicStatusCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TopicStatusCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TopicStatusCondition() = delete;

  /**
   * @brief A destructor for spraybot_behavior_tree::TopicStatusCondition
   */
  ~TopicStatusCondition() override;

  /**
   * @brief Callback function for status topic
   * @param msg Shared pointer to std::msg::Bool::SharedPtr message
   */
  void onStatusReceived(const typename std_msgs::msg::Bool::SharedPtr msg);

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
      BT::InputPort<std::string>("topic_name", std::string("status"), "Status topic name"),
      BT::InputPort<bool>("inital_status", false, "Inital status")
    };
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  std::string topic_name_;
  // Last status
  std::atomic<bool> status_;
  // Listen to status
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
};

}  // namespace spraybot_behavior_tree
