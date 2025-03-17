cmake_minimum_required(VERSION 3.5)
project(flex_bt_engine CXX)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  behaviortree_cpp
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
)

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
)

ament_export_dependencies(${dependencies})

ament_package()
