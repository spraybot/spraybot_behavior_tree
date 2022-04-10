// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <string>

#include "spraybot_behavior_tree/plugins/tick_rate_limiter.hpp"

namespace spraybot_behavior_tree
{

TickRateLimiter::TickRateLimiter(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(true)
{
  start_ = std::chrono::high_resolution_clock::now();
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

BT::NodeStatus TickRateLimiter::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  if (first_time_ || seconds.count() >= period_) {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();
    start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace spraybot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<spraybot_behavior_tree::TickRateLimiter>("TickRateLimiter");
}
