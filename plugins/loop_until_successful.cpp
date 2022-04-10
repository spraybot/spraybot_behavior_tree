/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include "spraybot_behavior_tree/plugins/loop_until_successful.hpp"

namespace spraybot_behavior_tree
{

LoopSuccessNode::LoopSuccessNode(const std::string & name, const BT::NodeConfiguration & config)
: DecoratorNode(name, config)
{
}

void LoopSuccessNode::halt()
{
  DecoratorNode::halt();
}

BT::NodeStatus LoopSuccessNode::tick()
{

  setStatus(BT::NodeStatus::RUNNING);

  BT::NodeStatus child_state;
  do {
    child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::SUCCESS:
        {
          haltChild();
          return BT::NodeStatus::SUCCESS;
        }

      case BT::NodeStatus::FAILURE:
        {
          haltChild();
        }
        break;

      case BT::NodeStatus::RUNNING:
        {
          return BT::NodeStatus::RUNNING;
        }

      default:
        {
          throw BT::LogicError("A child node must never return IDLE");
        }
    }
  } while (child_state != BT::NodeStatus::SUCCESS);

  return BT::NodeStatus::FAILURE;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::LoopSuccessNode>("LoopUntilSuccessful");
}
