cmake_minimum_required(VERSION 3.5)
project(flex_bt_server)

find_package(ament_cmake REQUIRED)
find_package(flex_bt_msgs REQUIRED)
find_package(flex_bt_engine REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_util REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(dwb_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_bt_navigator
  nav2_util
  nav_2d_utils
  nav_msgs
  behaviortree_cpp_v3
  dwb_core
  geometry_msgs
  flex_bt_msgs
  flex_bt_engine
)

include_directories(
	include
)

add_executable(${PROJECT_NAME}_bt_server_executor_node
	src/bt_server_executor_node.cpp
	src/bt_server_executor.cpp
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
