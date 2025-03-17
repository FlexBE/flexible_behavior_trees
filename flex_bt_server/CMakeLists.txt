cmake_minimum_required(VERSION 3.10)
project(flex_bt_server)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bondcpp REQUIRED)
find_package(flex_bt_msgs REQUIRED)
find_package(flex_bt_engine REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  behaviortree_cpp
  bondcpp
  flex_bt_msgs
  flex_bt_engine
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
  include
)

add_executable(${PROJECT_NAME}_bt_server_executor_node
  src/bt_server_executor_node.cpp
  src/bt_server_executor.cpp
  src/lifecycle_node.cpp
  src/node_thread.cpp
)
ament_target_dependencies(${PROJECT_NAME}_bt_server_executor_node
  ${dependencies})

install(TARGETS ${library_name}
                ${actions}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    ${PROJECT_NAME}_bt_server_executor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
