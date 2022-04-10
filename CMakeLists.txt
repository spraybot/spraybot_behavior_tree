cmake_minimum_required(VERSION 3.16)
project(spraybot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

set(dependencies
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  rclcpp
  robot_localization
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  lifecycle_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  nav2_core
  tf2_ros
)

set(executable_name hypervisor)
set(library_name ${executable_name}_core)


add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

add_library(${library_name} SHARED
  src/hypervisor.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})


list(APPEND 
    plugin_libs
    next_goal
    convert_ll2goal
    topic_status_condition
    publish_string
    publish_goal
    loop_until_successful
    tick_rate_limiter
    transition_lifecycle_node
)

foreach(bt_plugin ${plugin_libs})
  add_library(${bt_plugin}_bt_node SHARED plugins/${bt_plugin}.cpp)
  ament_target_dependencies(${bt_plugin}_bt_node ${dependencies})
  target_compile_definitions(${bt_plugin}_bt_node PRIVATE BT_PLUGIN_EXPORT)
endforeach()

list(TRANSFORM plugin_libs APPEND "_bt_node")

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

install(FILES spraybot_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable copyright and license checks
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable cpplint and manually call it with filters later
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS "-legal/copyright" "-build/header_guard" "-build/include_order")
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
