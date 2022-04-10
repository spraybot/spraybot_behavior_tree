/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "spraybot_behavior_tree/plugins/publish_goal.hpp"

namespace spraybot_behavior_tree
{

PublishGoal::PublishGoal(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  rclcpp::PublisherOptions pub_option;
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",
    rclcpp::SystemDefaultsQoS(), pub_option);

  RCLCPP_DEBUG(
    node_->get_logger(), "Initialized an PublishGoal BT node publishing goal");
}

PublishGoal::~PublishGoal()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down PublishGoal BT node");
}

BT::NodeStatus PublishGoal::tick()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();

  std::string frame_name;
  getInput("frame", frame_name);
  msg.header.frame_id = frame_name;

  std::vector<double> goal_pose;
  getInput("goal_pose", goal_pose);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal_pose[2]);

  msg.pose.position.x = goal_pose[0];
  msg.pose.position.y = goal_pose[1];
  msg.pose.orientation = tf2::toMsg(q);
  goal_pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::PublishGoal>("PublishGoal");
}
