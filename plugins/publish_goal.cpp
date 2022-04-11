/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "nav2_util/robot_utils.hpp"
#include <tf2/time.h>

#include "spraybot_behavior_tree/plugins/publish_goal.hpp"

using namespace std::chrono_literals;
namespace spraybot_behavior_tree
{

PublishGoal::PublishGoal(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

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
  geometry_msgs::msg::PoseStamped msg, msg_transformed;
  msg.header.stamp = node_->now();

  std::string goal_frame;
  getInput("goal_frame", goal_frame);
  msg.header.frame_id = goal_frame;

  std::vector<double> goal_pose;
  getInput("goal_pose", goal_pose);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal_pose[2]);

  msg.pose.position.x = goal_pose[0];
  msg.pose.position.y = goal_pose[1];
  msg.pose.orientation = tf2::toMsg(q);

  std::string publishing_frame;
  getInput("publishing_frame", publishing_frame);
  tf_->lookupTransform(publishing_frame, goal_frame, tf2::TimePointZero, 1000ms);
  nav2_util::transformPoseInTargetFrame(msg, msg_transformed, *tf_, publishing_frame);

  goal_pub_->publish(msg_transformed);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::PublishGoal>("PublishGoal");
}
