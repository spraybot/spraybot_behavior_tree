/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <nav2_behavior_tree/ros_topic_logger.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace spraybot_behavior_tree
{
class Hypervisor : public nav2_util::LifecycleNode
{
public:
  explicit Hypervisor(const std::string & node_name);

  ~Hypervisor() {}

  /**
   * @brief Configures member variables
   * Initializes action server for, builds behavior tree from xml file,
   * and calls user-defined onConfigure.
   * @return bool true on SUCCESS and false on FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates action server
   * @return bool true on SUCCESS and false on FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates action server
   * @return bool true on SUCCESS and false on FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Resets member variables
   * @return bool true on SUCCESS and false on FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void run();

  /**
   * @brief Replace current BT with another one
   * @param bt_xml_filename The file containing the new BT, uses default filename if empty
   * @return bool true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is maintained
   */
  bool loadBehaviorTree(const std::string & bt_xml_filename = "");

  /**
   * @brief Getter function for BT Blackboard
   * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
   */
  BT::Blackboard::Ptr getBlackboard() const
  {
    return blackboard_;
  }

  /**
   * @brief Get navigator's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */
  std::string getDefaultBTFilepath();

  /**
   * @brief Getter function for current BT XML filename
   * @return string Containing current BT XML filename
   */
  std::string getCurrentBTFilename() const
  {
    return current_bt_xml_filename_;
  }

  /**
   * @brief Getter function for default BT XML filename
   * @return string Containing default BT XML filename
   */
  std::string getDefaultBTFilename() const
  {
    return default_bt_xml_filename_;
  }

  /**
   * @brief Function to halt the current tree. It will interrupt the execution of RUNNING nodes
   * by calling their halt() implementation (only for Async nodes that may return RUNNING)
   */
  void haltTree()
  {
    tree_.rootNode()->halt();
  }

  /**
   * @brief Add groot monitor to publish BT status changes
   * @param tree BT to monitor
   * @param publisher_port ZMQ publisher port for the Groot monitor
   * @param server_port ZMQ server port for the Groot monitor
   * @param max_msg_per_second Maximum number of messages that can be sent per second
   */
  void addGrootMonitoring(
    BT::Tree & tree,
    uint16_t publisher_port,
    uint16_t server_port,
    uint16_t max_msg_per_second = 25);

  /**
   * @brief Reset groot monitor
   */
  void resetGrootMonitor();

protected:
  // Node name
  std::string node_name_;

  // Our action server implements the template action
  // Behavior Tree to be executed when goal is received
  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  static inline std::unique_ptr<BT::PublisherZMQ> groot_monitor_;

  // The XML file that cointains the Behavior Tree to create
  std::string current_bt_xml_filename_;
  std::string default_bt_xml_filename_;

  // The wrapper class for the BT functionality
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("Hypervisor")};

  // To publish BT logs
  std::unique_ptr<nav2_behavior_tree::RosTopicLogger> topic_logger_;

  // Duration for each iteration of BT execution
  std::chrono::milliseconds bt_loop_duration_;

  // Default timeout value while waiting for response from a server
  std::chrono::milliseconds default_server_timeout_;
};

}  // namespace spraybot_behavior_tree

// #include <nav2_behavior_tree/bt_action_server_impl.hpp>  // NOLINT(build/include_order)
