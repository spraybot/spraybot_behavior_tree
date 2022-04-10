/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <string>

#include "spraybot_behavior_tree/plugins/convert_ll2goal.hpp"

namespace spraybot_behavior_tree
{

ConvertLL2Goal::ConvertLL2Goal(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<robot_localization::srv::FromLL>(service_node_name, conf)
{
}

void ConvertLL2Goal::on_tick()
{
  geometry_msgs::msg::PoseStamped ll_goal;
  getInput("ll_goal", ll_goal);

  request_->ll_point.latitude = ll_goal.pose.position.x;
  request_->ll_point.longitude = ll_goal.pose.position.y;
  request_->ll_point.altitude = ll_goal.pose.position.z;
}

BT::NodeStatus ConvertLL2Goal::on_completion()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("ll_goal", goal);
  goal.pose.position = future_result_.get()->map_point;
  setOutput("goal", goal);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::ConvertLL2Goal>("ConvertLL2Goal");
}
