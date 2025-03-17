// Copyright 2023 Christopher Newport University
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Christopher Newport University nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef FLEX_BT_SERVER__BT_SERVER_EXECUTOR_H_
#define FLEX_BT_SERVER__BT_SERVER_EXECUTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"

// #include "behaviortree_cpp/loggers/groot2_publisher.h"
// #include "flex_bt_engine/behavior_tree_engine.hpp"


#include "flex_bt_server/bt_action_server.h"
#include "flex_bt_server/lifecycle_node.hpp"
#include "flex_bt_server/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace flex_bt
{

/**
   * @class flex_bt::BtServerExecutor
   * @brief Executes the BT Action Server with defined callbacks
   */
class BtServerExecutor : public flex_bt::LifecycleNode
{
public:
  /**
     * @brief A constructor for flex_bt::BtServerExecutor class
     */
  explicit BtServerExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
     * @brief A destructor forflex_bt::BtServerExecutor class
     */
  ~BtServerExecutor();


protected:
  /**
     * @brief Configures member variables
     *
     * Initializes bt server executor
     * "goal_sub"; and builds behavior tree from xml file.
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
  flex_bt::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
     * @brief Activates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
  flex_bt::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
     * @brief Deactivates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
  flex_bt::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
     * @brief Resets member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
  flex_bt::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
     * @brief Called when in shutdown state
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
  flex_bt::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Behavior Tree and BtExecute Action Servers
  using BtExecute = flex_bt_msgs::action::BtExecute;
  std::unique_ptr<flex_bt::BtActionServer<BtExecute>> bt_action_server_;

  using BtSetData = flex_bt_msgs::action::BtSetData;
  using BtGetData = flex_bt_msgs::action::BtGetData;

  /**
     * @brief NavigateToPose action server received goal callback
     */
  bool goalReceived(BtExecute::Goal::ConstSharedPtr goal);

  /**
     * @brief NavigateToPose action server main loop execution callback
     */
  void onLoop();

  /**
     * @brief NavigateToPose action server prempt callback
     */
  void onPreempt(BtExecute::Goal::ConstSharedPtr goal);

  /**
     * @brief NavigateToPose action server callback after finishing execution
     */
  void completionCallback(BtExecute::Result::SharedPtr result, flex_bt::BtStatus status);

  /**
     * @brief Goal pose initialization on the blackboard
     */
  bool initializeGoalPose(BtExecute::Goal::ConstSharedPtr goal);

  // The XML fiñe that cointains the default Behavior Tree to create
  std::string default_bt_xml_filename_;

  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Metrics for feedback
  rclcpp::Time start_time_;
  std::string robot_frame_;
  std::string global_frame_;
  std::string action_server_name_;
  std::string bt_server_name_;
  std::string bt_set_data_name_;
  std::string bt_get_data_name_;
  double transform_tolerance_;

  std::string goal_blackboard_id_;
  std::string path_blackboard_id_;
  std::string result_id_;
  std::string result_msg_type_;

  std::vector<geometry_msgs::msg::Point> points;
  std::vector<geometry_msgs::msg::PointStamped> points_stamped;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};
}  // namespace flex_bt

#endif  // FLEX_BT_SERVER__BT_SERVER_EXECUTOR_H_
