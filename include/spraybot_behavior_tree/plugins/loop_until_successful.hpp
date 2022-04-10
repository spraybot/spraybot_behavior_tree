/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <behaviortree_cpp_v3/decorator_node.h>

namespace spraybot_behavior_tree
{

class LoopSuccessNode : public BT::DecoratorNode
{
public:
  LoopSuccessNode(const std::string & name, const BT::NodeConfiguration & config);

  virtual ~LoopSuccessNode() override = default;

  static BT::PortsList providedPorts()
  {
    return {};
  }

  virtual void halt() override;

private:
  virtual BT::NodeStatus tick() override;
};

}
