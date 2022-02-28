// Copyright (c) 2020 Sarthak Mittal
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

#ifndef FLEX_BT_SERVER__BT_ACTION_SERVER_H_
#define FLEX_BT_SERVER__BT_ACTION_SERVER_H_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "flex_bt_engine/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace flex_bt {
  template<class ActionT>
  class BtActionServer {
  public:
    using ActionServer = nav2_util::SimpleActionServer<ActionT>;

    using BtLoad = flex_bt_msgs::action::BtLoad;
    using BtLoadServer = nav2_util::SimpleActionServer<BtLoad>;

    typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
    typedef std::function<void ()> OnLoopCallback;
    typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
    typedef std::function<void (typename ActionT::Result::SharedPtr)> OnCompletionCallback;

    explicit BtActionServer(
      const std::shared_ptr<nav2_util::LifecycleNode> parent,
      const std::string & action_name,
      const std::string & bt_action_name,
      const std::vector<std::string> & plugin_lib_names,
      const std::string & default_bt_xml_filename,
      OnGoalReceivedCallback on_goal_received_callback,
      OnLoopCallback on_loop_callback,
      OnPreemptCallback on_preempt_callback = {},
      OnCompletionCallback on_completion_callback = {})
      :
        action_name_(action_name),
        bt_action_name_(bt_action_name),
        default_bt_xml_filename_(default_bt_xml_filename),
        plugin_lib_names_(plugin_lib_names),
        node_(parent),
        on_goal_received_callback_(on_goal_received_callback),
        on_loop_callback_(on_loop_callback),
        on_preempt_callback_(on_preempt_callback),
        on_completion_callback_(on_completion_callback)
      {
        logger_ = node_->get_logger();
        clock_ = node_->get_clock();

        // Declare this node's parameters
        if (!node_->has_parameter("bt_loop_duration")) {
          node_->declare_parameter("bt_loop_duration", 10);
        }
        if (!node_->has_parameter("default_server_timeout")) {
          node_->declare_parameter("default_server_timeout", 20);
        }

        action_server_ = std::make_unique<ActionServer>(
          node_->get_node_base_interface(),
          node_->get_node_clock_interface(),
          node_->get_node_logging_interface(),
          node_->get_node_waitables_interface(),
          action_name_,
          std::bind(&BtActionServer<ActionT>::executeCallback, this));

        bt_server_ = std::make_unique<BtLoadServer>(
          node_->get_node_base_interface(),
          node_->get_node_clock_interface(),
          node_->get_node_logging_interface(),
          node_->get_node_waitables_interface(),
          bt_action_name_,
          std::bind(&BtActionServer<ActionT>::handleLoadingBTs, this));
      };

    ~BtActionServer() {};

    /**
     * @brief Configures member variables
     * Initializes action server for, builds behavior tree from xml file,
     * and calls user-defined onConfigure.
     * @return bool true on SUCCESS and false on FAILURE
     */
    bool on_configure() {
      std::string client_node_name = action_name_;
      std::replace(client_node_name.begin(), client_node_name.end(), '/', '_');
      auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args",
          "-r",
          std::string("__node:=") + std::string(node_->get_name()) + client_node_name + "_rclcpp_node",
          "--"});

      client_node_ = std::make_shared<rclcpp::Node>("_", options);

      // Get parameters for BT timeouts
      int timeout;
      node_->get_parameter("bt_loop_duration", timeout);
      bt_loop_duration_ = std::chrono::milliseconds(timeout);
      node_->get_parameter("default_server_timeout", timeout);
      default_server_timeout_ = std::chrono::milliseconds(timeout);

      // Create the class that registers our custom nodes and executes the BT
      bt_ = std::make_unique<flex_bt::BehaviorTreeEngine>(plugin_lib_names_);

      // Create the blackboard that will be shared by all of the nodes in the tree
      blackboard_ = BT::Blackboard::create();

      // Put items on the blackboard
      blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
      blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);
      blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
      return true;
    };

    /**
     * @brief Activates action server
     * @return bool true on SUCCESS and false on FAILURE
     */
    bool on_activate() {
      if (!loadBehaviorTree(default_bt_xml_filename_)) {
        RCLCPP_ERROR(logger_, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
        return false;
      }
      action_server_->activate();
      return true;
    }

    /**
     * @brief Deactivates action server
     * @return bool true on SUCCESS and false on FAILURE
     */
    bool on_deactivate() {
      action_server_->deactivate();
      return true;
    }

    /**
     * @brief Resets member variables
     * @return bool true on SUCCESS and false on FAILURE
     */
    bool on_cleanup() {
      client_node_.reset();
      action_server_.reset();
      plugin_lib_names_.clear();
      current_bt_xml_filename_.clear();
      blackboard_.reset();

      if (current_tree_ != nullptr) {
          bt_->haltAllActions(current_tree_->rootNode());
      }

      bt_.reset();
      for(std::map<std::string, BT::Tree*>::iterator itr = trees_.begin(); itr != trees_.end(); itr++) {
          delete itr->second;
      }
      trees_.clear();
      delete current_tree_;

      bt_.reset();
      return true;
    }

    /**
     * @brief Replace current BT with another one
     * @param bt_xml_filename The file containing the new BT, uses default filename if empty
     * @return bool true if the resulting BT correspond to the one in bt_xml_filename. false
     * if something went wrong, and previous BT is maintained
     */
    bool loadBehaviorTree(const std::string & bt_xml_filename = "") {
      auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

      // Check if the loaded filename before
      if (trees_.count(filename)) {
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
      BT::Tree* tree = new BT::Tree();
      *tree = bt_->createTreeFromText(xml_string, blackboard_);
      trees_[bt_xml_filename] = tree;
      current_bt_xml_filename_ = filename;

      RCLCPP_INFO(logger_, "Finished loading default bt");
      return true;
    }

    /**
     * @brief Getter function for BT Blackboard
     * @return BT::Blackboard::Ptr Shared pointer to current BT blackboard
     */
    BT::Blackboard::Ptr getBlackboard() const {
      return blackboard_;
    }

    /**
     * @brief Getter function for current BT XML filename
     * @return string Containing current BT XML filename
     */
    std::string getCurrentBTFilename() const {
      return current_bt_xml_filename_;
    }

    /**
     * @brief Getter function for default BT XML filename
     * @return string Containing default BT XML filename
     */
    std::string getDefaultBTFilename() const {
      return default_bt_xml_filename_;
    }

    /**
     * @brief Wrapper function to accept pending goal if a preempt has been requested
     * @return Shared pointer to pending action goal
     */
    const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal() {
      return action_server_->accept_pending_goal();
    }

    /**
     * @brief Wrapper function to terminate pending goal if a preempt has been requested
     */
    void terminatePendingGoal() {
      action_server_->terminate_all();
    }

    /**
     * @brief Wrapper function to get current goal
     * @return Shared pointer to current action goal
     */
    const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const {
      return action_server_->get_current_goal();
    }

    /**
     * @brief Wrapper function to succeed current goal
     */
    void succeedCurrent(typename std::shared_ptr<typename ActionT::Result> result) {
      action_server_->succeeded_current(result);
    }

    /**
     * @brief Wrapper function to fail or cancel current goal
     */
    void terminateCurrent(typename std::shared_ptr<typename ActionT::Result> result) {
      action_server_->terminate_current(result);
    }

    /**
     * @brief Wrapper function to publish action feedback
     */
    void publishFeedback(typename std::shared_ptr<typename ActionT::Feedback> feedback) {
      action_server_->publish_feedback(feedback);
    }

    /**
     * @brief Getter function for the current BT tree
     * @return BT::Tree Current behavior tree
     */
    BT::Tree* getCurrentTree() const {
      return current_tree_;
    }

    /**
     * @brief Function to halt the current tree. It will interrupt the execution of RUNNING nodes
     * by calling their halt() implementation (only for Async nodes that may return RUNNING)
     */
    void haltTree() {
      current_tree_->rootNode()->halt();
    }

  protected:
    /**
     * @brief Action server callback
     */
    void executeCallback() {
      if (!on_goal_received_callback_(action_server_->get_current_goal())) {
        action_server_->terminate_current();
        return;
      }

      current_bt_xml_filename_ = action_server_->get_current_goal()->behavior_tree == "" ?
        default_bt_xml_filename_ : action_server_->get_current_goal()->behavior_tree;

      current_tree_ = trees_[current_bt_xml_filename_];

      if (current_tree_ == nullptr) {
        RCLCPP_ERROR(logger_, "BT not created with file %. Navigation canceled.",
                     current_bt_xml_filename_.c_str());
        action_server_->terminate_current();
        return;
      }

      // Functions used in executing BT
      auto is_canceling = [&]() {
          if (action_server_ == nullptr) {
            RCLCPP_DEBUG(logger_, "Action server unavailable. Canceling.");
            return true;
          }
          if (!action_server_->is_server_active()) {
            RCLCPP_DEBUG(logger_, "Action server is inactive. Canceling.");
            return true;
          }
          return action_server_->is_cancel_requested();
        };

      auto on_loop = [&]() {
          if (action_server_->is_preempt_requested() && on_preempt_callback_) {
            on_preempt_callback_(action_server_->get_current_goal());
          }
          else {
            on_loop_callback_();
          }
        };

      // Execute the BT
      flex_bt::BtStatus status = bt_->run(current_tree_, on_loop, is_canceling, bt_loop_duration_);
      bt_->haltAllActions(current_tree_->rootNode());

      // Give server an opportunity to populate the result message or simple give
      // an indication that the action is complete.
      auto result = std::make_shared<typename ActionT::Result>();

      if (on_completion_callback_) {
          on_completion_callback_(result);
      }

      switch (status) {
        case flex_bt::BtStatus::SUCCEEDED:
          RCLCPP_INFO(logger_, "Goal succeeded");
          action_server_->succeeded_current(result);
          break;

        case flex_bt::BtStatus::FAILED:
          RCLCPP_ERROR(logger_, "Goal failed");
          action_server_->terminate_current(result);
          break;

        case flex_bt::BtStatus::CANCELED:
          RCLCPP_INFO(logger_, "Goal canceled");
          action_server_->terminate_current(result);
          break;
      }
    }

    /**
     *  @bried BT Server callback to load a list of BTs
     */
    void handleLoadingBTs() {
      auto goal = bt_server_->get_current_goal();

      for (int i = 0; i < goal->filepaths.size(); i++) {
        RCLCPP_INFO(logger_, "Loading file: %s", goal->filepaths[i].c_str());

        if (!trees_.count(goal->filepaths[i])) {
          // Read the input BT XML from the specified file into a string
          std::ifstream xml_file(goal->filepaths[i]);

          if (!xml_file.good()) {
            RCLCPP_ERROR(logger_, "Couldn't open input XML file: %s", goal->filepaths[i].c_str());
          }

          auto xml_string = std::string(
            std::istreambuf_iterator<char>(xml_file),
            std::istreambuf_iterator<char>());

          // Create the Behavior Tree from the XML input
          BT::Tree* tree = new BT::Tree();
          *tree = bt_->createTreeFromText(xml_string, blackboard_);
          trees_[goal->filepaths[i]] = tree;
          RCLCPP_INFO(logger_, "Add BT from file: %s", goal->filepaths[i].c_str());
        }
        std::shared_ptr<flex_bt_msgs::action::BtLoad::Result> result =
          std::make_shared<flex_bt_msgs::action::BtLoad::Result>();
        result->code = flex_bt_msgs::action::BtLoad::Result::SUCCESS;
        bt_server_->succeeded_current(result);
      }
    }

    std::string action_name_;
    std::string bt_action_name_;

    std::unique_ptr<ActionServer> action_server_;
    std::unique_ptr<BtLoadServer> bt_server_;

    std::unique_ptr<flex_bt::BehaviorTreeEngine> bt_;
    BT::Blackboard::Ptr blackboard_;

    BT::Tree* current_tree_;
    std::map<std::string, BT::Tree*> trees_;

    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    std::vector<std::string> plugin_lib_names_;

    rclcpp::Node::SharedPtr client_node_;
    std::shared_ptr<nav2_util::LifecycleNode> node_;

    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Logger logger_{rclcpp::get_logger("BtActionServer")};

    std::chrono::milliseconds bt_loop_duration_;
    std::chrono::milliseconds default_server_timeout_;

    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;
  };
}

// #include <flex_bt_server/bt_action_server_impl.h>
#endif
