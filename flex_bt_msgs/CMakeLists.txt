cmake_minimum_required(VERSION 3.5)
project(flex_bt_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BtLoad.action"
  "action/BtExecute.action"
  "action/BtSetData.action"
  "action/BtGetData.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
