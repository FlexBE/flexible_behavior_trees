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
  BtServerExecutor::BtServerExecutor()
  : nav2_util::LifecycleNode("bt_server", "", false),
    start_time_(0)
  {
    const std::vector<std::string> plugin_libs = {
      "flex_nav_get_path_action_bt_node",
      "flex_nav_follow_path_action_bt_node",
      "flex_nav_follow_topic_action_bt_node",
      "flex_nav_clear_costmap_action_bt_node",
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

    // Declare this node's parameters
    declare_parameter("default_bt_xml_filename");
    declare_parameter("plugin_lib_names", plugin_libs);
    declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter("global_frame", std::string("map"));
    declare_parameter("robot_base_frame", std::string("base_link"));
    declare_parameter("odom_topic", std::string("odom"));
    declare_parameter("action_server_name", "bt_executor");
    declare_parameter("bt_server_name", "bt_loader");
    declare_parameter("enable_groot_monitoring", true);
    declare_parameter("groot_zmq_publisher_port", 1666);
    declare_parameter("groot_zmq_server_port", 1667);
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

    // Get the libraries to pull plugins from
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_base_frame").as_string();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();

    get_parameter("action_server_name", action_server_name_);
    get_parameter("bt_server_name", bt_server_name_);
    get_parameter("plugin_lib_names", plugin_lib_names_);
    get_parameter("default_bt_xml_filename", default_bt_xml_filename_);

    declare_parameter("goal_blackboard_id", std::string("goal"));

    // Configure an array of goals and other userdata

    goal_blackboard_id_ = get_parameter("goal_blackboard_id").as_string();
    if (!has_parameter("path_blackboard_id")) {
      declare_parameter("path_blackboard_id", std::string("path"));
    }
    path_blackboard_id_ = get_parameter("path_blackboard_id").as_string();

    bt_action_server_ = std::make_unique<flex_bt::BtActionServer<BtExecute>>(
      shared_from_this(), action_server_name_, bt_server_name_, plugin_lib_names_,
      default_bt_xml_filename_,
      std::bind(&BtServerExecutor::goalReceived, this, std::placeholders::_1),
      std::bind(&BtServerExecutor::onLoop, this),
      std::bind(&BtServerExecutor::onPreempt, this, std::placeholders::_1),
      std::bind(&BtServerExecutor::completionCallback, this, std::placeholders::_1)
    );

    bt_action_server_->on_configure();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerExecutor::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating");
    bt_action_server_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn BtServerExecutor::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Deactivating");
    bt_action_server_->on_deactivate();
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

    initializeGoalPose(goal);

    return true;
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
      bt_action_server_->terminatePendingGoal();
    }
  }

  void BtServerExecutor::completionCallback(BtExecute::Result::SharedPtr result) {
    result->code = BtExecute::Result::SUCCESS;
    bt_action_server_->succeedCurrent(result);
    RCLCPP_INFO(get_logger(), "Success");
  }

  void BtServerExecutor::initializeGoalPose(BtExecute::Goal::ConstSharedPtr goal) {
    // Reset state for new action feedback
    start_time_ = now();
    auto blackboard = bt_action_server_->getBlackboard();

    // Set goal posees if needed
    if (goal->goal_id != "" && goal->goal_poses.size() > 0) {
      if (goal->goal_poses.size() == 1) {
        blackboard->set<geometry_msgs::msg::PoseStamped>(goal->goal_id, goal->goal_poses.at(0));
      }
      else {
        blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>(goal->goal_id, goal->goal_poses);
      }
    }
  }
}
