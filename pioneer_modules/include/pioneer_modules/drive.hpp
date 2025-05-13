// Copyright (c) 2025 Alberto J. Tudela Roldán
// Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef PIONEER_MODULES__DRIVE_HPP_
#define PIONEER_MODULES__DRIVE_HPP_

// Aria
#include <Aria/ArRobot.h>

// C++
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

// PIONEER
#include "pioneer_core/module.hpp"
#include "pioneer_msgs/msg/bumper_state.hpp"
#include "pioneer_msgs/srv/enable_motors.hpp"

namespace pioneer_modules
{

/**
 * @class pioneer_modules::Drive
 * @brief Module for interfacing all related to drive: odometry, motor controller, state etc.
 *
 */
class Drive : public pioneer_core::Module
{
public:
  /**
   * @brief Construct for pioneer_modules::Drive
   */
  Drive() = default;

  /**
   * @brief Destructor for pioneer_modules::Drive
   */
  ~Drive() override = default;

  /**
   * @brief Configure the module.
   *
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param robot pointer to the ArRobot object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::weak_ptr<ArRobot> robot) override;

  /**
   * @brief Cleanup the module state machine.
   */
  void cleanup() override;

  /**
   * @brief Activate the module state machine.
   */
  void activate() override;

  /**
   * @brief Deactivate the module state machine.
   */
  void deactivate() override;

protected:
  /**
   * @brief Callback executed when the odometry data is received.
   */
  void odometryDataCallback();

  /**
   * @brief Callback executed when the bumper data is received.
   */
  void bumperDataCallback();

  /**
   * @brief Callback executed when velocity command is received.
   *
   * @param msg Velocity command
   */
  void velocityCommandCallback(const geometry_msgs::msg::Twist & msg);

  /**
   * @brief Callback executed after a timeout when no velocity command is received.
   */
  void velocityWatchdogCallback();

  /**
   * @brief Enable motors service callback.
   *
   * @param request Enable motors request.
   * @param response Enable motors response.
   * @return bool If the motors were successfully enabled.
   */
  bool enableMotors(
    const std::shared_ptr<pioneer_msgs::srv::EnableMotors::Request> request,
    std::shared_ptr<pioneer_msgs::srv::EnableMotors::Response> response);

  /**
   * @brief Callback executed when a parameter change is detected.
   * @param event ParameterEvent message.
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Convert Aria ArPose to ROS Odometry.
   *
   * @param pose Pose from Aria.
   * @param linear_vel_x Linear velocity in x direction [mm/s].
   * @param linear_vel_y Linear velocity in y direction [mm/s].
   * @param angular_vel_z Angular velocity in z direction [deg/s].
   * @return nav_msgs::msg::Odometry Odometry for ROS.
   */
  nav_msgs::msg::Odometry ariaToRosOdometry(
    const ArPose & pose, double linear_vel_x, double linear_vel_y, double angular_vel_z);

  /**
   * @brief Convert Aria ArPose to ROS TF.
   *
   * @param pose Pose from Aria.
   * @return geometry_msgs::msg::TransformStamped Transform for ROS.
   */
  geometry_msgs::msg::TransformStamped ariaToRosTf(const ArPose & pose);

  // Plugin related
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("Drive")};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Aria related
  std::unique_ptr<ArFunctorC<Drive>> odometry_callback_functor_;
  std::unique_ptr<ArFunctorC<Drive>> bumper_callback_functor_;

  // ROS Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pioneer_msgs::msg::BumperState>>
  front_bumper_pub_, rear_bumper_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odometry_pub_;

  // ROS Subscribers
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> cmd_vel_stamped_sub_;

  // ROS Services
  std::shared_ptr<rclcpp::Service<pioneer_msgs::srv::EnableMotors>> enable_motors_service_;

  std::string robot_base_frame_, odom_frame_, odom_topic_;
  bool is_stamped_;
  rclcpp::Time last_vel_received_;
  rclcpp::TimerBase::SharedPtr cmd_vel_watchdog_timer_;
  rclcpp::Duration vel_timeout_{0, 0};

  // Bumpers
  pioneer_msgs::msg::BumperState front_bumper_state_, rear_bumper_state_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_;
};

}  // namespace pioneer_modules

#endif  // PIONEER_MODULES__DRIVE_HPP_
