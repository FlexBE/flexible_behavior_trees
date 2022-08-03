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


#include "nav2_bt_navigator/bt_navigator.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>
#include <exception>
#include <regex>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "flex_bt_server/bt_server_executor.h"

namespace flex_bt {
  BtServerExecutor::BtServerExecutor(const rclcpp::NodeOptions & options)
  : nav2_util::LifecycleNode("bt_server", "", options),
    start_time_(0)
  {
    const std::vector<std::string> plugin_libs = {
      "nav2_back_up_action_bt_node",
      "nav2_spin_action_bt_node",
      "nav2_wait_action_bt_node",
      "nav2_goal_reached_condition_bt_node",
      "nav2_goal_updated_condition_bt_node",
      "nav2_rate_controller_bt_node",
      "nav2_recovery_node_bt_node",
      "nav2_pipeline_sequence_bt_node",
      "nav2_goal_updater_node_bt_node",
      "nav2_compute_path_to_pose_action_bt_node",
      "nav2_follow_path_action_bt_node",
      "nav2_clear_costmap_service_bt_node",
      "nav2_is_stuck_condition_bt_node",
      "nav2_initial_pose_received_condition_bt_node",
      "nav2_reinitialize_global_localization_service_bt_node",
      "nav2_distance_controller_bt_node",
      "nav2_speed_controller_bt_node",
      "nav2_truncate_path_action_bt_node",
      "nav2_round_robin_node_bt_node",
      "nav2_transform_available_condition_bt_node",
      "nav2_time_expired_condition_bt_node",
      "nav2_distance_traveled_condition_bt_node",
      "nav2_is_battery_low_condition_bt_node",
    };

    declare_parameter("default_bt_xml_filename", rclcpp::ParameterValue(std::string("")));
    declare_parameter("plugin_lib_names", rclcpp::ParameterValue(plugin_libs));
    declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
    declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    declare_parameter("odom_topic", rclcpp::ParameterValue(std::string("odom")));
    declare_parameter("action_server_name", rclcpp::ParameterValue(std::string("bt_executor")));
    declare_parameter("bt_server_name", rclcpp::ParameterValue(std::string("bt_loader")));
    declare_parameter("bt_set_data_name", rclcpp::ParameterValue(std::string("bt_set_data")));
    declare_parameter("bt_get_data_name", rclcpp::ParameterValue(std::string("bt_get_data")));
    declare_parameter("enable_groot_monitoring", rclcpp::ParameterValue(true));
    declare_parameter("groot_zmq_publisher_port", rclcpp::ParameterValue(1666));
    declare_parameter("groot_zmq_server_port", rclcpp::ParameterValue(1667));
  }

  BtServerExecutor::~BtServerExecutor() {
    RCLCPP_INFO(get_logger(), "Destroying");
  }

  nav2_util::CallbackReturn BtServerExecutor::on_configure(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring BT Server");

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

    get_parameter("action_server_name", action_server_name_);
    get_parameter("bt_server_name", bt_server_name_);
    get_parameter("bt_set_data_name", bt_set_data_name_);
    get_parameter("bt_get_data_name", bt_get_data_name_);
    get_parameter("plugin_lib_names", plugin_lib_names_);
    get_parameter("default_bt_xml_filename", default_bt_xml_filename_);
    get_parameter("global_frame", global_frame_);
    get_parameter("robot_base_frame", robot_frame_);
    get_parameter("transform_tolerance", transform_tolerance_);

    declare_parameter("goal_blackboard_id", std::string("goal"));
    goal_blackboard_id_ = get_parameter("goal_blackboard_id").as_string();
    if (!has_parameter("path_blackboard_id")) {
      declare_parameter("path_blackboard_id", std::string("path"));
    }
    path_blackboard_id_ = get_parameter("path_blackboard_id").as_string();

    bt_action_server_ = std::make_unique<flex_bt::BtActionServer<BtExecute>>(
      shared_from_this(), action_server_name_, bt_server_name_, bt_set_data_name_,
      bt_get_data_name_, plugin_lib_names_, default_bt_xml_filename_,
      std::bind(&BtServerExecutor::goalReceived, this, std::placeholders::_1),
      std::bind(&BtServerExecutor::onLoop, this),
      std::bind(&BtServerExecutor::onPreempt, this, std::placeholders::_1),
      std::bind(&BtServerExecutor::completionCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    bt_action_server_->on_configure();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerExecutor::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating");
    bt_action_server_->on_activate();
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerExecutor::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating");
    bt_action_server_->on_deactivate();
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerExecutor::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    tf_listener_.reset();
    tf_.reset();
    bt_action_server_->on_cleanup();
    bt_action_server_.reset();
    plugin_lib_names_.clear();
    RCLCPP_INFO(get_logger(), "Completed Cleaning up");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerExecutor::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  bool BtServerExecutor::goalReceived(BtExecute::Goal::ConstSharedPtr goal) {
    auto bt_xml_filename = goal->behavior_tree;

    if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
      RCLCPP_ERROR(
        get_logger(), "BT file not found: %s. Navigation canceled.",
        bt_xml_filename.c_str());
      return false;
    }

    bt_action_server_->setupGroot();
    return initializeGoalPose(goal);
  }

  void BtServerExecutor::onLoop() {
    auto feedback_msg = std::make_shared<BtExecute::Feedback>();

    std::function<std::string(const BT::TreeNode*)> recursiveTraverse;

    recursiveTraverse = [&recursiveTraverse](const BT::TreeNode* node) -> std::string {
        if (!node) {
            return "";
        }

        std::string active_nodes = "";

        if (node->status() == BT::NodeStatus::RUNNING) {
          active_nodes += node->name() + ", ";
        }

        if (auto control = dynamic_cast<const BT::ControlNode*>(node)) {
            for (const auto& child : control->children())
            {
                active_nodes += recursiveTraverse(child) + ", ";
            }
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node)) {
            active_nodes += recursiveTraverse(decorator->child()) + ", ";
        }

        return active_nodes;
    };

    feedback_msg->active_nodes = recursiveTraverse(bt_action_server_->getCurrentTree()->rootNode());
    std::regex pattern(" ,");
    feedback_msg->active_nodes = std::regex_replace(feedback_msg->active_nodes, pattern, "");
    feedback_msg->active_nodes = feedback_msg->active_nodes.substr(0, feedback_msg->active_nodes.length() - 2);

    nav2_util::getCurrentPose(feedback_msg->current_location, *tf_, global_frame_, robot_frame_, transform_tolerance_);
    feedback_msg->execution_time = now() - start_time_;
    bt_action_server_->publishFeedback(feedback_msg);
  }

  void BtServerExecutor::onPreempt(BtExecute::Goal::ConstSharedPtr goal) {
    RCLCPP_INFO(get_logger(), "Received goal preemption request");
    std::shared_ptr<BtExecute::Result> result =
      std::make_shared<BtExecute::Result>();
    result->code = BtExecute::Result::CANCEL;
    bt_action_server_->terminateCurrent(result);

    if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
      (goal->behavior_tree.empty() &&
      bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
    {
      // if pending goal requests the same BT as the current goal, accept the pending goal
      // if pending goal has an empty behavior_tree field, it requests the default BT file
      // accept the pending goal if the current goal is running the default BT file
      initializeGoalPose(bt_action_server_->acceptPendingGoal());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Preemption request was rejected since the requested BT XML file is not the same "
        "as the one that the current goal is executing. Preemption with a new BT is invalid "
        "since it would require cancellation of the previous goal instead of true preemption."
        "\nCancel the current goal and send a new action request if you want to use a "
        "different BT XML file. For now, continuing to track the last goal until completion.");
      std::shared_ptr<flex_bt_msgs::action::BtExecute::Result> abort =
        std::make_shared<flex_bt_msgs::action::BtExecute::Result>();

      abort->code = flex_bt_msgs::action::BtExecute::Result::CANCEL;
      bt_action_server_->terminatePendingGoal(abort);
    }
  }

  void BtServerExecutor::completionCallback(BtExecute::Result::SharedPtr result, flex_bt::BtStatus status) {
    auto blackboard = bt_action_server_->getBlackboard();

    switch (status) {
      case flex_bt::BtStatus::SUCCEEDED:
        result->code = BtExecute::Result::SUCCESS;
        break;

      case flex_bt::BtStatus::FAILED:
        result->code = BtExecute::Result::FAILURE;
        break;

      case flex_bt::BtStatus::CANCELED:
        result->code = BtExecute::Result::CANCEL;
        break;
    }

    std::vector<std::string> converted = bt_action_server_->getBlackboardDataVector(result_id_, result_msg_type_);

    if (converted.size() == 0) {
      std::string converted_prim = bt_action_server_->getPrimitiveBlackboardData(result_id_, result_msg_type_);

      if (converted_prim.length() > 0) {
        converted.push_back(converted_prim);
      }
      else {
        converted.push_back(bt_action_server_->getBlackboardData(result_id_, result_msg_type_));
      }
    }

    result->result_data = converted;

    bt_action_server_->succeedCurrent(result);
  }

  bool BtServerExecutor::initializeGoalPose(BtExecute::Goal::ConstSharedPtr goal) {
    // Reset state for new action feedback
    start_time_ = now();

    try {
      if (goal->goal_id != "" && goal->msg_type != "" && goal->msg_data.size() > 0) {

        // Check for primitive types first
        bool set_primitive = bt_action_server_->setPrimitiveBlackboardData(goal->goal_id, goal->msg_type, goal->msg_data.at(0));

        if (!set_primitive) {

            if (goal->msg_data.size() > 1 || goal->msg_type == "path" || goal->msg_type == "Path") {
              bt_action_server_->setBlackboardDataVector(goal->goal_id, goal->msg_type, goal->msg_data);
            }  else {
              bt_action_server_->setBlackboardData(goal->goal_id, goal->msg_type, goal->msg_data.at(0));
            }
        }
      }

      if (goal->result_id != "") {
        RCLCPP_INFO(get_logger(), "Flexible BT Server %s - initialized goal pose with result  %s",
                                  get_name(), goal->result_id.c_str());

        result_id_ = goal->result_id;
        result_msg_type_ = goal->result_msg_type;
      } else {
        RCLCPP_INFO(get_logger(), "Flexible BT Server %s - initialized goal pose with empty result",
                                  get_name());
      }
      return true;
    }
    catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Failed to set given data. Exception: %s",
        ex.what());

      return false;
    }
  }
}
