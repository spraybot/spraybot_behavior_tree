/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <string>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "spraybot_behavior_tree/plugins/transition_lifecycle_node.hpp"

namespace spraybot_behavior_tree
{

TransitionLifecycleNode::TransitionLifecycleNode(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<lifecycle_msgs::srv::ChangeState>(service_node_name, conf)
{
}

// TODO: (shrijitsingh99) Add shutdown transition
std::unordered_map<std::string, unsigned int> TransitionLifecycleNode::transition_map = {
    {"create", lifecycle_msgs::msg::Transition::TRANSITION_CREATE},
    {"configure", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE},
    {"activate", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE},
    {"deactivate", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE},
    {"cleanup", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP}
};

void TransitionLifecycleNode::on_tick()
{
  std::string transition_label;
  getInput("transition_label", transition_label);
  auto transition = lifecycle_msgs::msg::Transition();
  transition.id = TransitionLifecycleNode::transition_map[transition_label];
  transition.label = transition_label;
  request_->transition = transition;
}

BT::NodeStatus TransitionLifecycleNode::on_completion()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::TransitionLifecycleNode>(
    "TransitionLifecycleNode");
}
