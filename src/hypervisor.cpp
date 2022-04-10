/*
 * Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include "spraybot_behavior_tree/hypervisor.hpp"

namespace spraybot_behavior_tree
{

Hypervisor::Hypervisor(const std::string & node_name)
: nav2_util::LifecycleNode(node_name, "", false),
  node_name_(node_name)
{
  logger_ = get_logger();
  clock_ = get_clock();

  plugin_lib_names_ = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    // "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    // "nav2_globally_updated_goal_condition_bt_node",
    // "nav2_is_path_valid_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    // "nav2_controller_cancel_bt_node",
    // "nav2_wait_cancel_bt_node",
    // "nav2_spin_cancel_bt_node",
    // "nav2_back_up_cancel_bt_node",
    "topic_status_condition_bt_node",
    "publish_string_bt_node",
    "publish_goal_bt_node",
    "loop_until_successful_bt_node",
    "tick_rate_limiter_bt_node",
    "transition_lifecycle_node_bt_node"
  };

  declare_parameter("plugin_lib_names", plugin_lib_names_);
}

nav2_util::CallbackReturn Hypervisor::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring %s", node_name_.c_str());
  std::string client_node_name = node_name_;
  std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
  // Use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r",
      std::string("__node:=") + std::string(get_name()) + "_rclcpp_node",
      "--"});

  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  // Declare this node's parameters
  if (!has_parameter("bt_loop_duration")) {
    declare_parameter("bt_loop_duration", 10);
  }

  // Get parameters for BT timeouts
  int timeout;
  get_parameter("bt_loop_duration", timeout);
  bt_loop_duration_ = std::chrono::milliseconds(timeout);

  default_bt_xml_filename_ = getDefaultBTFilepath();

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);    // NOLINT
  blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);    // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);    // NOLINT
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);    // NOLINT

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Hypervisor::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating %s", node_name_.c_str());
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Hypervisor::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating %s", node_name_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Hypervisor::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up %s", node_name_.c_str());
  client_node_.reset();
  // Reset the tf listener before the tf buffer
  tf_listener_.reset();
  tf_.reset();
  topic_logger_.reset();
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Hypervisor::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down %s", node_name_.c_str());
  return nav2_util::CallbackReturn::SUCCESS;
}

void Hypervisor::run()
{
  auto is_canceling = [&]() {
      return false;
    };

  auto on_loop = [&]() {
      topic_logger_->flush();
    };

  nav2_behavior_tree::BtStatus rc;
  do {
    RCLCPP_INFO(logger_, "Starting new iteration for hypervisor");
    rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
  } while (rc == nav2_behavior_tree::BtStatus::SUCCEEDED);
  RCLCPP_INFO(logger_, "Hypervisor finished running!");
  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_.rootNode());
}

std::string
Hypervisor::getDefaultBTFilepath()
{
  std::string default_bt_xml_filename;
  if (!has_parameter("default_hypervisor_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("spraybot_behavior_tree");
    declare_parameter<std::string>(
      "default_hypervisor_bt_xml",
      pkg_share_dir +
      "/behavior_trees/hypervisor.xml");
  }

  get_parameter("default_hypervisor_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool Hypervisor::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Empty filename is default for backward compatibility
  auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

  // Use previous BT if it is the existing one
  if (current_bt_xml_filename_ == filename) {
    RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", filename.c_str());
    return false;
  }

  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  // Create the Behavior Tree from the XML input
  tree_ = bt_->createTreeFromText(xml_string, blackboard_);
  topic_logger_ = std::make_unique<nav2_behavior_tree::RosTopicLogger>(client_node_, tree_);

  current_bt_xml_filename_ = filename;
  return true;
}
}
