/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <string>
#include <vector>

#include "spraybot_behavior_tree/plugins/next_goal.hpp"

namespace spraybot_behavior_tree
{

NextGoal::NextGoal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
}

inline BT::NodeStatus NextGoal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  std::vector<geometry_msgs::msg::PoseStamped> goals;

  getInput("input_goals", goals);

  if (goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal = goals.front();
  goals.erase(goals.begin());

  setOutput("goal", goal);
  setOutput("output_goals", goals);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::NextGoal>("NextGoal");
}
