#ifndef FLEX_BT_SERVER__BT_SERVER_EXECUTOR_H_
#define FLEX_BT_SERVER__BT_SERVER_EXECUTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/basic_types.h"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "flex_bt_msgs/action/bt_load.hpp"
#include "flex_bt_msgs/action/bt_execute.hpp"
#include "flex_bt_server/bt_action_server.h"

namespace flex_bt {
  /**
   * @class nav2_bt_navigator::BtNavigator
   * @brief An action server that uses behavior tree for navigating a robot to its
   * goal position.
   */
  class BtServerExecutor : public nav2_util::LifecycleNode {
  public:
    /**
     * @brief A constructor for nav2_bt_navigator::BtNavigator class
     */
    BtServerExecutor();

    /**
     * @brief A destructor for nav2_bt_navigator::BtNavigator class
     */
    ~BtServerExecutor();

  protected:
    /**
     * @brief Configures member variables
     *
     * Initializes action server for "NavigationToPose"; subscription to
     * "goal_sub"; and builds behavior tree from xml file.
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Activates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Deactivates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Resets member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Called when in shutdown state
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    // Behavior Tree and NavigateToPose Action Servers
    using BtExecute = flex_bt_msgs::action::BtExecute;
    std::unique_ptr<flex_bt::BtActionServer<BtExecute>> bt_action_server_;

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
    void completionCallback(BtExecute::Result::SharedPtr result);

    /**
     * @brief Goal pose initialization on the blackboard
     */
    void initializeGoalPose(BtExecute::Goal::ConstSharedPtr goal);

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
    double transform_tolerance_;

    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;
  };
}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
