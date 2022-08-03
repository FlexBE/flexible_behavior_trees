/*******************************************************************************
 *  Copyright (c) 2022
 *  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
 *  Christopher Newport University
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *       POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef FLEX_BT_SERVER__BT_ACTION_SERVER_H_
#define FLEX_BT_SERVER__BT_ACTION_SERVER_H_

#include <memory>
#include <string>
#include <fstream>
#include <set>
#include <exception>
#include <vector>

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "flex_bt_msgs/action/bt_load.hpp"
#include "flex_bt_msgs/action/bt_execute.hpp"
#include "flex_bt_msgs/action/bt_set_data.hpp"
#include "flex_bt_msgs/action/bt_get_data.hpp"
#include "flex_bt_engine/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "flex_bt_server/string_conversions.h"

namespace flex_bt {
  template<class ActionT>
  class BtActionServer {
  public:
    using ActionServer = nav2_util::SimpleActionServer<ActionT>;

    using BtLoad = flex_bt_msgs::action::BtLoad;
    using BtLoadServer = nav2_util::SimpleActionServer<BtLoad>;

    using BtSetData = flex_bt_msgs::action::BtSetData;
    using BtSetDataServer = nav2_util::SimpleActionServer<BtSetData>;

    using BtGetData = flex_bt_msgs::action::BtGetData;
    using BtGetDataServer = nav2_util::SimpleActionServer<BtGetData>;

    typedef std::function<bool (typename ActionT::Goal::ConstSharedPtr)> OnGoalReceivedCallback;
    typedef std::function<void ()> OnLoopCallback;
    typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
    typedef std::function<void (typename ActionT::Result::SharedPtr, flex_bt::BtStatus status)> OnCompletionCallback;

    explicit BtActionServer(
      const std::shared_ptr<nav2_util::LifecycleNode> parent,
      const std::string & action_name,
      const std::string & bt_server_name,
      const std::string & bt_set_data_name,
      const std::string & bt_get_data_name,
      const std::vector<std::string> & plugin_lib_names,
      const std::string & default_bt_xml_filename,
      OnGoalReceivedCallback on_goal_received_callback,
      OnLoopCallback on_loop_callback,
      OnPreemptCallback on_preempt_callback,
      OnCompletionCallback on_completion_callback)
      :
        action_name_(action_name),
        bt_server_name_(bt_server_name),
        bt_set_data_name_(bt_set_data_name),
        bt_get_data_name_(bt_get_data_name),
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
          bt_server_name_,
          std::bind(&BtActionServer<ActionT>::handleLoadingBTs, this));

        bt_set_data_server_ = std::make_unique<BtSetDataServer>(
          node_->get_node_base_interface(),
          node_->get_node_clock_interface(),
          node_->get_node_logging_interface(),
          node_->get_node_waitables_interface(),
          bt_set_data_name_,
          std::bind(&BtActionServer<ActionT>::setGivenData, this));

        bt_get_data_server_ = std::make_unique<BtGetDataServer>(
          node_->get_node_base_interface(),
          node_->get_node_clock_interface(),
          node_->get_node_logging_interface(),
          node_->get_node_waitables_interface(),
          bt_get_data_name_,
          std::bind(&BtActionServer<ActionT>::getRequestedData, this));
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

      int timeout;
      node_->get_parameter("bt_loop_duration", timeout);
      bt_loop_duration_ = std::chrono::milliseconds(timeout);
      node_->get_parameter("default_server_timeout", timeout);
      default_server_timeout_ = std::chrono::milliseconds(timeout);

      // Create the class that registers our custom nodes and executes the BT
      bt_ = std::make_unique<flex_bt::BehaviorTreeEngine>(plugin_lib_names_);

      blackboard_ = BT::Blackboard::create();
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
      bt_server_->activate();
      bt_set_data_server_->activate();
      bt_get_data_server_->activate();

      return true;
    }

    /**
     * @brief Deactivates action server
     * @return bool true on SUCCESS and false on FAILURE
     */
    bool on_deactivate() {
      action_server_->deactivate();
      bt_server_->deactivate();
      bt_set_data_server_->deactivate();
      bt_get_data_server_->deactivate();

      return true;
    }

    /**
     * @brief Resets member variables
     * @return bool true on SUCCESS and false on FAILURE
     */
    bool on_cleanup() {
      client_node_.reset();
      action_server_.reset();
      bt_server_.reset();
      bt_set_data_server_.reset();
      bt_get_data_server_.reset();
      plugin_lib_names_.clear();
      current_bt_xml_filename_.clear();
      blackboard_.reset();

      bt_->resetGrootMonitor();

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
      bt_->resetGrootMonitor();

      // Check if the loaded filename before
      if (trees_.count(filename)) {
        RCLCPP_DEBUG(logger_, "BT will not be reloaded as the given xml is already loaded");
        current_tree_ = trees_[filename];
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
      current_tree_ = tree;
      current_bt_xml_filename_ = filename;

      RCLCPP_INFO(logger_, "Finished loading BT");
      return true;
    }

    /**
     * @brief sets up Groot monitoring for the current BT
     */
    void setupGroot() {
      if (node_->get_parameter("enable_groot_monitoring").as_bool()) {
        uint16_t zmq_publisher_port = node_->get_parameter("groot_zmq_publisher_port").as_int();
        uint16_t zmq_server_port = node_->get_parameter("groot_zmq_server_port").as_int();
        try {
          bt_->addGrootMonitoring(current_tree_, zmq_publisher_port, zmq_server_port);
        } catch (const std::logic_error & e) {
          RCLCPP_ERROR(logger_, "ZMQ already enabled, Error: %s", e.what());
        }
      }
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
    void terminatePendingGoal(typename std::shared_ptr<typename ActionT::Result> result) {
      action_server_->terminate_all(result);
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


    /**
     * Store a primitive type in the blackboard
     */
    bool setPrimitiveBlackboardData(std::string goal_id, std::string msg_type, std::string msg_data) {
      if (msg_type == "string") {
        blackboard_->set<std::string>(goal_id, msg_data);
        return true;
      }
      else if (msg_type == "char") {
        auto converted_char = BT::convertFromString<char*>(msg_data);
        blackboard_->set<char*>(goal_id, converted_char);
        return true;
      }
      else if (msg_type == "bool") {
        auto boolean = BT::convertFromString<bool>(msg_data);
        blackboard_->set<bool>(goal_id, boolean);
        return true;
      }
      else if (msg_type == "float") {
        auto converted_float = BT::convertFromString<float>(msg_data);
        blackboard_->set<float>(goal_id, converted_float);
        return true;
      }
      else if (msg_type == "int") {
        auto converted_int = BT::convertFromString<int>(msg_data);
        blackboard_->set<int>(goal_id, converted_int);
        return true;
      }
      else if (msg_type == "int[]") {
        auto converted_ints = BT::convertFromString<std::vector<int>>(msg_data);
        blackboard_->set<std::vector<int>>(goal_id, converted_ints);
        return true;
      }
      else if (msg_type == "double") {
        auto converted_double = BT::convertFromString<double>(msg_data);
        blackboard_->set<double>(goal_id, converted_double);
        return true;
      }
      else if (msg_type == "double[]") {
        auto converted_doubles = BT::convertFromString<std::vector<double>>(msg_data);
        blackboard_->set<std::vector<double>>(goal_id, converted_doubles);
        return true;
      }

      // Not a primitive base type - handle elsewhere
      return false;
    }

    /**
     * Store a ROS message types in the blackboard
     */
    void setBlackboardData(std::string goal_id, std::string msg_type, std::string msg_data) {
      if (msg_type == "point" || msg_type == "Point") {
        auto point = BT::convertFromString<geometry_msgs::msg::Point>(msg_data);
        blackboard_->set<geometry_msgs::msg::Point>(goal_id, point);
      }
      else if (msg_type == "point_stamped" || msg_type == "pointstamped" || msg_type == "PointStamped") {
        auto point_stamped = BT::convertFromString<geometry_msgs::msg::PointStamped>(msg_data);
        blackboard_->set<geometry_msgs::msg::PointStamped>(goal_id, point_stamped);
      }
      else if (msg_type == "quaternion" || msg_type == "Quaternion") {
        auto quaternion = BT::convertFromString<geometry_msgs::msg::Quaternion>(msg_data);
        blackboard_->set<geometry_msgs::msg::Quaternion>(goal_id, quaternion);
      }
      else if (msg_type == "pose_stamped" || msg_type == "posestamped" || msg_type == "PoseStamped") {
        auto pose_stamped = BT::convertFromString<geometry_msgs::msg::PoseStamped>(msg_data);
        blackboard_->set<geometry_msgs::msg::PoseStamped>(goal_id, pose_stamped);
      }
      else if (msg_type == "twist_stamped" || msg_type == "twiststamped" || msg_type == "TwistStamped") {
        auto twist_stamped = BT::convertFromString<geometry_msgs::msg::TwistStamped>(msg_data);
        blackboard_->set<geometry_msgs::msg::TwistStamped>(goal_id, twist_stamped);
      } else {
        RCLCPP_WARN(logger_, "Flexible BT Server %s setBlackboardData - unknown message type  %s",
                                  node_->get_name(), msg_type.c_str());
      }
    }

    /**
     * Store a vector or list of ROS messages
     */
    void setBlackboardDataVector(std::string goal_id, std::string msg_type, std::vector<std::string> msg_data) {
      nav_msgs::msg::Path path;
      std::vector<geometry_msgs::msg::Point> points;
      std::vector<geometry_msgs::msg::PointStamped> points_stamped;
      std::vector<geometry_msgs::msg::PoseStamped> poses;
      std::vector<geometry_msgs::msg::TwistStamped> twists_stamped;

      bool path_msg = msg_type == "path" || msg_type == "Path";
      bool point_msg = msg_type == "point[]" || msg_type == "Point[]";
      bool pointstamped_msg = msg_type == "point_stamped[]" || msg_type == "pointstamped[]" || msg_type == "PointStamped[]";
      bool posestamped_msg = msg_type == "pose_stamped[]" || msg_type == "posestamped[]" || msg_type == "PoseStamped[]";
      bool twiststamped_msg = msg_type == "twist_stamped[]" || msg_type == "twiststamped[]" || msg_type == "TwistStamped[]";

      for (int i = 0; i < msg_data.size(); i++) {
        if (path_msg && i == 0) {
          auto header = BT::convertFromString<std_msgs::msg::Header>(msg_data.at(i));
          path.header = header;
        }
        else if (point_msg) {
          auto point = BT::convertFromString<geometry_msgs::msg::Point>(msg_data.at(i));
          points.push_back(point);
        }
        else if (pointstamped_msg) {
          auto point_stamped = BT::convertFromString<geometry_msgs::msg::PointStamped>(msg_data.at(i));
          points_stamped.push_back(point_stamped);
        }
        else if (path_msg || posestamped_msg) {
          auto pose = BT::convertFromString<geometry_msgs::msg::PoseStamped>(msg_data.at(i));
          poses.push_back(pose);
        }
        else if (twiststamped_msg) {
          auto twist = BT::convertFromString<geometry_msgs::msg::TwistStamped>(msg_data.at(i));
          twists_stamped.push_back(twist);
        }
      }

      if (path_msg) {
        path.poses = poses;
        blackboard_->set<nav_msgs::msg::Path>(goal_id, path);
      }
      else if (point_msg) {
        blackboard_->set<std::vector<geometry_msgs::msg::Point>>(goal_id, points);
      }
      else if (pointstamped_msg) {
        blackboard_->set<std::vector<geometry_msgs::msg::PointStamped>>(goal_id, points_stamped);
      }
      else if (posestamped_msg) {
        blackboard_->set<std::vector<geometry_msgs::msg::PoseStamped>>(goal_id, poses);
      }
      else if (twiststamped_msg) {
        blackboard_->set<std::vector<geometry_msgs::msg::TwistStamped>>(goal_id, twists_stamped);
      } else {
        RCLCPP_WARN(logger_, "Flexible BT Server %s setBlackboardDataVector - unknown message type  %s",
                                  node_->get_name(), msg_type.c_str());
      }
    }

    /**
     * Get a primitve type from the blackboard and concert it to a string
     */
    std::string getPrimitiveBlackboardData(std::string data_id, std::string msg_type) {
      std::string converted = "";

      if (msg_type == "string") {
        converted = blackboard_->get<std::string>(data_id) + ";";
      }
      else if (msg_type == "char") {
        std::string converted_str(blackboard_->get<char*>(data_id));
        converted = converted_str + ";";
      }
      else if (msg_type == "bool") {
        converted = blackboard_->get<bool>(data_id) ? "True;" : "False;";
      }
      else if (msg_type == "float") {
        converted = std::to_string(blackboard_->get<float>(data_id)) + ";";
      }
      else if (msg_type == "int") {
        converted = std::to_string(blackboard_->get<int>(data_id)) + ";";
      }
      else if (msg_type == "int[]") {
        auto ints = blackboard_->get<std::vector<int>>(data_id);
        for (int i = 0; i < ints.size(); i++) {
          converted += std::to_string(ints.at(i)) + ";";
        }
      }
      else if (msg_type == "double") {
        auto data = blackboard_->get<double>(data_id);
        converted = std::to_string(blackboard_->get<double>(data_id)) + ";";
      }
      else if (msg_type == "double[]") {
        auto doubles = blackboard_->get<std::vector<double>>(data_id);
        for (int i = 0; i < doubles.size(); i++) {
          converted += std::to_string(doubles.at(i)) + ";";
        }
      }

      return converted;
    }

    /**
     * Gets a ROS message from the blackboard and converts it to a string
     */
    std::string getBlackboardData(std::string data_id, std::string msg_type) {
      std::string converted = "";

      if (msg_type == "point" || msg_type == "Point") {
        auto point = blackboard_->get<geometry_msgs::msg::Point>(data_id);
        converted =  BT::convertPointToString(point);
      }
      else if (msg_type == "point_stamped" || msg_type == "pointstamped" || msg_type == "PointStamped") {
        auto point_stamped = blackboard_->get<geometry_msgs::msg::PointStamped>(data_id);
        converted = BT::convertPointStampedToString(point_stamped);
      }
      else if (msg_type == "quaternion" || msg_type == "Quaternion") {
        auto quaternion = blackboard_->get<geometry_msgs::msg::Quaternion>(data_id);
        converted =  BT::convertQuaternionToString(quaternion);
      }
      else if (msg_type == "pose_stamped" || msg_type == "posestamped" || msg_type == "PoseStamped") {
        auto pose_stamped = blackboard_->get<geometry_msgs::msg::PoseStamped>(data_id);
        converted =  BT::convertPoseStampedToString(pose_stamped);
      }
      else if (msg_type == "twist_stamped" || msg_type == "twiststamped" || msg_type == "TwistStamped") {
        auto twist_stamped = blackboard_->get<geometry_msgs::msg::TwistStamped>(data_id);
        converted =  BT::convertTwistStampedToString(twist_stamped);
      }

      return converted;
    }

    /**
     * Gets a vector or list of ROS messages from the blackboard
     */
    std::vector<std::string> getBlackboardDataVector(std::string data_id, std::string msg_type) {
      std::vector<std::string> converted;

      if (msg_type == "path" || msg_type == "Path") {
        auto path = blackboard_->get<nav_msgs::msg::Path>(data_id);

        converted.push_back(BT::convertHeaderToString(path.header));
        for (int i = 1; i < path.poses.size(); i++) {
          converted.push_back(BT::convertPoseStampedToString(path.poses.at(i)));
        }
      }
      else if (msg_type == "point[]" || msg_type == "Point[]") {
        auto points = blackboard_->get<std::vector<geometry_msgs::msg::Point>>(data_id);

        for (int i = 0; i < points.size(); i++) {
          converted.push_back(BT::convertPointToString(points.at(i)));
        }
      }
      else if (msg_type == "point_stamped[]" || msg_type == "pointstamped[]" || msg_type == "PointStamped[]") {
        auto points_stamped = blackboard_->get<std::vector<geometry_msgs::msg::PointStamped>>(data_id);

        for (int i = 0; i < points_stamped.size(); i++) {
          converted.push_back(BT::convertPointStampedToString(points_stamped.at(i)));
        }
      }
      else if (msg_type == "pose_stamped[]" || msg_type == "posestamped[]" || msg_type == "PoseStamped[]") {
        auto poses = blackboard_->get<std::vector<geometry_msgs::msg::PoseStamped>>(data_id);

        for (int i = 0; i < poses.size(); i++) {
          converted.push_back(BT::convertPoseStampedToString(poses.at(i)));
        }
      }
      else if (msg_type == "twist_stamped[]" || msg_type == "twiststamped[]" || msg_type == "TwistStamped[]") {
        auto twists = blackboard_->get<std::vector<geometry_msgs::msg::TwistStamped>>(data_id);

        for (int i = 0; i < twists.size(); i++) {
          converted.push_back(BT::convertTwistStampedToString(twists.at(i)));
        }
      }

      return converted;
    }

  protected:
    /**
     * @brief Action server callback
     */
    void executeCallback() {
      if (!on_goal_received_callback_(action_server_->get_current_goal())) {
        std::shared_ptr<flex_bt_msgs::action::BtExecute::Result> abort =
          std::make_shared<flex_bt_msgs::action::BtExecute::Result>();

        abort->code = flex_bt_msgs::action::BtExecute::Result::FAILURE;
        action_server_->terminate_current(abort);
        return;
      }

      current_bt_xml_filename_ = action_server_->get_current_goal()->behavior_tree == "" ?
        default_bt_xml_filename_ : action_server_->get_current_goal()->behavior_tree;

      current_tree_ = trees_[current_bt_xml_filename_];

      if (current_tree_ == nullptr) {
        RCLCPP_ERROR(logger_, "BT not created with file %s", current_bt_xml_filename_.c_str());
        action_server_->terminate_current();
        return;
      }

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

      // After executing send back the action server's result message
      auto result = std::make_shared<typename ActionT::Result>();
      if (on_completion_callback_) {
          on_completion_callback_(result, status);
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

    /**
     *  @bried BT Server callback to load a list of BTs
     */
    void setGivenData() {
      try {
        auto goal = bt_set_data_server_->get_current_goal();
        if (goal->goal_id != "" && goal->goal_msg_type != "" && goal->goal_msg_data.size() > 0) {
          bool set_primitive = setPrimitiveBlackboardData(goal->goal_id, goal->goal_msg_type, goal->goal_msg_data.at(0));

          if (!set_primitive) {

              if (goal->goal_msg_data.size() > 1 || goal->goal_msg_type == "path" || goal->goal_msg_type == "Path") {
                  setBlackboardDataVector(goal->goal_id, goal->goal_msg_type, goal->goal_msg_data);
              } else {
                  setBlackboardData(goal->goal_id, goal->goal_msg_type, goal->goal_msg_data.at(0));
              }
          }
        }

        std::shared_ptr<flex_bt_msgs::action::BtSetData::Result> result =
          std::make_shared<flex_bt_msgs::action::BtSetData::Result>();
        result->code = flex_bt_msgs::action::BtSetData::Result::SUCCESS;
        bt_set_data_server_->succeeded_current(result);
      }
      catch (const std::exception& ex) {
        RCLCPP_FATAL(logger_, "Failed to set given data. Exception: %s",
          ex.what());

        std::shared_ptr<flex_bt_msgs::action::BtSetData::Result> abort =
          std::make_shared<flex_bt_msgs::action::BtSetData::Result>();

        abort->code = flex_bt_msgs::action::BtSetData::Result::FAILURE;
        bt_set_data_server_->terminate_current(abort);
      }
    }

    /**
     *  @bried BT Server callback to load a list of BTs
     */
    void getRequestedData() {
      try {
        auto goal = bt_get_data_server_->get_current_goal();
        std::vector<std::string> converted = getBlackboardDataVector(goal->request_id, goal->request_msg_type);

        if (converted.size() == 0) {
          std::string converted_prim = getPrimitiveBlackboardData(goal->request_id, goal->request_msg_type);

          if (converted_prim.length() > 0) {
            converted.push_back(converted_prim);
          }
          else {
            converted.push_back(getBlackboardData(goal->request_id, goal->request_msg_type));
          }
        }

        std::shared_ptr<flex_bt_msgs::action::BtGetData::Result> result =
          std::make_shared<flex_bt_msgs::action::BtGetData::Result>();
        result->code = flex_bt_msgs::action::BtGetData::Result::SUCCESS;
        result->result_data = converted;

        bt_get_data_server_->succeeded_current(result);
      }
      catch (const std::exception& ex) {
        RCLCPP_FATAL(logger_, "Failed to get requested data. Exception: %s",
          ex.what());

        std::shared_ptr<flex_bt_msgs::action::BtGetData::Result> abort =
          std::make_shared<flex_bt_msgs::action::BtGetData::Result>();

        abort->code = flex_bt_msgs::action::BtGetData::Result::FAILURE;
        bt_get_data_server_->terminate_current(abort);
      }
    }

    std::string action_name_;
    std::string bt_server_name_;
    std::string bt_set_data_name_;
    std::string bt_get_data_name_;

    std::unique_ptr<ActionServer> action_server_;
    std::unique_ptr<BtLoadServer> bt_server_;
    std::unique_ptr<BtSetDataServer> bt_set_data_server_;
    std::unique_ptr<BtGetDataServer> bt_get_data_server_;

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

#endif
