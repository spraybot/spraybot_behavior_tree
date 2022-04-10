/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <memory>
#include <thread>

#include "spraybot_behavior_tree/hypervisor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<spraybot_behavior_tree::Hypervisor>("hypervisor");
  if (node->declare_parameter<bool>("autostart", true)) {
    node->configure();
    node->activate();
  }
  auto bt_thread = std::thread([&]() {node->run();});
  rclcpp::spin(node->get_node_base_interface());
  bt_thread.join();
  rclcpp::shutdown();

  return 0;
}
