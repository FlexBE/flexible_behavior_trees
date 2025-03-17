// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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

#include "flex_bt_engine/behavior_tree_engine.hpp"

#include <dlfcn.h>
#include <link.h>
#include <iostream>

#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"


void printLibraryInfo() {
   Dl_info dl_info;
    if (dladdr(reinterpret_cast<void *>(&BT::WildcardMatch), &dl_info) == 0) {
        std::cerr << "Failed to get BehaviorTree.CPP library information: "
        << dlerror() << std::flush << std::endl;
    } else {
        std::cout << "BehaviorTree.CPP Library Path: "
        << dl_info.dli_fname  << std::flush<< std::endl;
    }
}

namespace flex_bt
{


BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries) :
factory_()
{
  printLibraryInfo();
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    const auto p_name = loader.getOSName(p);
    try {
      factory_.registerFromPlugin(p_name);
    } catch (const std::exception & ex) {
      std::cout << "\x1b[95m  Error loading plugin "
       << p << " : " << std::endl << ex.what() << " ...\x1b[0m" << std::endl << std::flush;
    }
  }
}


BtStatus BehaviorTreeEngine::run(
  BT::Tree * tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;


  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    if (cancelRequested()) {
      tree->rootNode()->haltNode();
      return BtStatus::CANCELED;
    }

    try {
      result = tree->tickOnce();
    } catch (const std::exception & ex) {
      return BtStatus::FAILED;
    }

    onLoop();



    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string, BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path, BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

void BehaviorTreeEngine::addGrootMonitoring(BT::Tree * tree, uint16_t server_port)
{
  // This logger publish status changes using Groot2Publisher. Used by Groot
  groot2_monitor_ =
    std::make_unique<BT::Groot2Publisher>(*tree, server_port);
  std::cout << "Defined Groot2 publisher at port " << server_port << "!" << std::endl;
}

void BehaviorTreeEngine::resetGrootMonitor() {
  groot2_monitor_.reset();

  // RCLCPP_ERROR(node->get_logger(), "Not using Groot!");
  std::cout << "Not using Groot!" << std::endl;
  }

void BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  // this halt signal should propagate through the entire tree.
  root_node->haltNode();
  auto visitor = [](BT::TreeNode * node) {
    if (node->status() == BT::NodeStatus::RUNNING) {
      node->haltNode();
    }
  };
  BT::applyRecursiveVisitor(root_node, visitor);
}
}  // namespace flex_bt
