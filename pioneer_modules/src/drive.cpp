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

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pioneer_modules/drive.hpp"

namespace pioneer_modules
{

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;

void Drive::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::weak_ptr<ArRobot> robot)
{
  // Declare and read parameters
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();

  robot_ = robot.lock();
  if (!robot_) {
    throw std::runtime_error("Unable to lock robot!");
  }

  // Declare and read parameters
  declare_parameter_if_not_declared(
    node_, plugin_name_ + ".robot_base_frame",
    rclcpp::ParameterValue("base_link"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the base frame of the robot"));
  node_->get_parameter(plugin_name_ + ".robot_base_frame", robot_base_frame_);
  RCLCPP_INFO(logger_, "The parameter robot_base_frame is set to: [%s]", robot_base_frame_.c_str());

  declare_parameter_if_not_declared(
    node_, plugin_name_ + ".odom_frame",
    rclcpp::ParameterValue("odom"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the odometry frame"));
  node_->get_parameter(plugin_name_ + ".odom_frame", odom_frame_);
  RCLCPP_INFO(logger_, "The parameter odom_frame is set to: [%s]", odom_frame_.c_str());

  declare_parameter_if_not_declared(
    node_, plugin_name_ + ".odom_topic",
    rclcpp::ParameterValue("odom"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the odometry topic"));
  node_->get_parameter(plugin_name_ + ".odom_topic", odom_topic_);
  RCLCPP_INFO(logger_, "The parameter odom_topic is set to: [%s]", odom_topic_.c_str());

  declare_parameter_if_not_declared(
    node_, plugin_name_ + ".enable_stamped_cmd_vel",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable the stamped cmd_vel topic"));
  node_->get_parameter(plugin_name_ + ".enable_stamped_cmd_vel", is_stamped_);
  RCLCPP_INFO(
    logger_, "The parameter enable_stamped_cmd_vel is set to: [%s]",
      is_stamped_ ? "true" : "false");

  declare_parameter_if_not_declared(
    node_, plugin_name_ + ".publish_tf",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Publish the odometry as a tf2 transform"));
  node_->get_parameter(plugin_name_ + ".publish_tf", publish_tf_);
  RCLCPP_INFO(
    logger_, "The parameter publish_tf_ is set to: [%s]", publish_tf_ ? "true" : "false");

  int timeout = 0;
  declare_parameter_if_not_declared(
    node_, plugin_name_ + ".velocity_timeout",
    rclcpp::ParameterValue(20), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The interval in milliseconds to check for new velocity commands"));
  node_->get_parameter(plugin_name_ + ".velocity_timeout", timeout);
  RCLCPP_INFO(logger_, "The parameter velocity_timeout is set to: [%i]", timeout);
  vel_timeout_ = rclcpp::Duration::from_seconds(timeout / 1000.0);

  // Create ROS publishers
  auto latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  front_bumper_pub_ = node_->create_publisher<pioneer_msgs::msg::BumperState>(
    "front_bumper", latched_profile);
  rear_bumper_pub_ = node_->create_publisher<pioneer_msgs::msg::BumperState>(
    "rear_bumper", latched_profile);
  odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS());

  // Create Aria subscribers
  odometry_callback_functor_ =
    std::make_unique<ArFunctorC<Drive>>(this, &Drive::odometryDataCallback);
  bumper_callback_functor_ =
    std::make_unique<ArFunctorC<Drive>>(this, &Drive::bumperDataCallback);
  robot_->addSensorInterpTask("odometry", 100, odometry_callback_functor_.get());
  robot_->addSensorInterpTask("bumper", 100, bumper_callback_functor_.get());

  // Create ROS subscribers
  if (is_stamped_) {
    cmd_vel_stamped_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel",
      rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        velocityCommandCallback(msg->twist);
      });
  } else {
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        velocityCommandCallback(*msg);
      });
  }

  // Create ROS services
  enable_motors_service_ = node_->create_service<pioneer_msgs::srv::EnableMotors>(
    "drive/enable_motors", std::bind(&Drive::enableMotors, this, _1, _2));

  // Callback for monitor changes in parameters
  dyn_params_handler_ = node_->add_on_set_parameters_callback(
    std::bind(&Drive::dynamicParametersCallback, this, _1));

  // Initialize the transform broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  // Initialize the bumpers
  front_bumper_state_.bumpers.resize(robot_->getNumFrontBumpers());
  rear_bumper_state_.bumpers.resize(robot_->getNumRearBumpers());
}

void Drive::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type pioneer_module::Drive", plugin_name_.c_str());
  front_bumper_pub_.reset();
  rear_bumper_pub_.reset();
  odometry_pub_.reset();
  cmd_vel_sub_.reset();
  cmd_vel_stamped_sub_.reset();
  enable_motors_service_.reset();
  tf_broadcaster_.reset();
  cmd_vel_watchdog_timer_.reset();
}

void Drive::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type pioneer_module::Drive", plugin_name_.c_str());
  front_bumper_pub_->on_activate();
  rear_bumper_pub_->on_activate();
  odometry_pub_->on_activate();

  robot_->enableMotors();

  cmd_vel_watchdog_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&Drive::velocityWatchdogCallback, this));
}

void Drive::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type pioneer_module::Drive", plugin_name_.c_str());

  robot_->disableMotors();

  front_bumper_pub_->on_deactivate();
  rear_bumper_pub_->on_deactivate();
  odometry_pub_->on_deactivate();
}

rcl_interfaces::msg::SetParametersResult Drive::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_BOOL) {
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".robot_base_frame") {
        robot_base_frame_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter base_frame is set to: [%s]", robot_base_frame_.c_str());
      } else if (name == plugin_name_ + ".odom_frame") {
        odom_frame_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter odom_frame is set to: [%s]", odom_frame_.c_str());
      } else if (name == plugin_name_ + ".odom_topic") {
        odom_topic_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter odom_topic is set to: [%s]", odom_topic_.c_str());
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".velocity_timeout") {
        int timeout = parameter.as_int();
        vel_timeout_ = rclcpp::Duration::from_seconds(timeout / 1000.0);
        RCLCPP_INFO(logger_, "The parameter velocity_timeout is set to: [%i]", timeout);
      }
    }
  }

  result.successful = true;
  return result;
}

void Drive::odometryDataCallback()
{
  ArPose pose = robot_->getPose();
  double linear_vel_x = robot_->getVel();
  double linear_vel_y = robot_->getLatVel();
  double angular_vel_z = robot_->getRotVel();

  auto odom_msg = ariaToRosOdometry(pose, linear_vel_x, linear_vel_y, angular_vel_z);
  odometry_pub_->publish(odom_msg);

  // Publish the TF
  if (publish_tf_) {
    auto tf_msg = ariaToRosTf(pose);
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void Drive::bumperDataCallback()
{
  // getStallValue returns 2 bytes with stall bit and bumper bits,
  // packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot_->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  front_bumper_state_.header.frame_id = robot_base_frame_;
  front_bumper_state_.header.stamp = clock_->now();

  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i = 0; i < front_bumper_state_.bumpers.size(); i++) {
    front_bumper_state_.bumpers[i] = (front_bumpers & (1 << (i + 1))) == 0 ? 0 : 1;
  }

  rear_bumper_state_.header.frame_id = robot_base_frame_;
  rear_bumper_state_.header.stamp = clock_->now();

  // Rear bumpers have reverse order (rightmost is LSB)
  for (unsigned int i = 0; i < rear_bumper_state_.bumpers.size(); i++) {
    rear_bumper_state_.bumpers[i] =
      (rear_bumpers & (1 << (rear_bumper_state_.bumpers.size() - i))) == 0 ? 0 : 1;
  }

  front_bumper_pub_->publish(front_bumper_state_);
  rear_bumper_pub_->publish(rear_bumper_state_);
}

void Drive::velocityCommandCallback(const geometry_msgs::msg::Twist & msg)
{
  last_vel_received_ = clock_->now();
  robot_->lock();
  robot_->setVel(msg.linear.x * 1e3);
  if (robot_->hasLatVel()) {
    robot_->setLatVel(msg.linear.y * 1e3);
  }
  robot_->setRotVel(msg.angular.z * 180 / M_PI);
  robot_->unlock();
}

void Drive::velocityWatchdogCallback()
{
  // If no velocity command has been received, stop the robot
  if (clock_->now() - last_vel_received_ > vel_timeout_) {
    robot_->lock();
    robot_->setVel(0);
    if (robot_->hasLatVel()) {
      robot_->setLatVel(0);
    }
    robot_->setRotVel(0);
    robot_->unlock();
  }
}

bool Drive::enableMotors(
  const std::shared_ptr<pioneer_msgs::srv::EnableMotors::Request> request,
  std::shared_ptr<pioneer_msgs::srv::EnableMotors::Response>/*response*/)
{
  robot_->lock();
  if (request->enable) {
    if (robot_->isEStopPressed()) {
      RCLCPP_WARN(logger_,
          "Enable motors requested, but robot also has E-Stop button pressed. "
          "Motors will not be enabled.");
    }
    robot_->enableMotors();
  } else {
    robot_->disableMotors();
  }
  robot_->unlock();
  return true;
}

nav_msgs::msg::Odometry Drive::ariaToRosOdometry(
  const ArPose & pose, double linear_vel_x, double linear_vel_y, double angular_vel_z)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.header.stamp = clock_->now();
  odom_msg.child_frame_id = robot_base_frame_;

  // Set the position
  odom_msg.pose.pose.position.x = pose.getX() / 1000.0;
  odom_msg.pose.pose.position.y = pose.getY() / 1000.0;
  odom_msg.pose.pose.orientation =
    tf2::toMsg(tf2::Quaternion({0, 0, 1}, pose.getTh() * M_PI / 180.0));

  // Set the velocity
  // Aria returns the velocity in mm/s and the angle in degrees
  odom_msg.twist.twist.linear.x = linear_vel_x / 1000.0;
  odom_msg.twist.twist.linear.y = linear_vel_y / 1000.0;
  odom_msg.twist.twist.angular.x = angular_vel_z * M_PI / 180.0;

  return odom_msg;
}

geometry_msgs::msg::TransformStamped Drive::ariaToRosTf(const ArPose & pose)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.header.stamp = clock_->now();
  tf_msg.child_frame_id = robot_base_frame_;
  tf_msg.transform.translation.x = pose.getX() / 1000.0;
  tf_msg.transform.translation.y = pose.getY() / 1000.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation =
    tf2::toMsg(tf2::Quaternion({0, 0, 1}, pose.getTh() * M_PI / 180.0));
  return tf_msg;
}

}  // namespace pioneer_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(pioneer_modules::Drive, pioneer_core::Module)
