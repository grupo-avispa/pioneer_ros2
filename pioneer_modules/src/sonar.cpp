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


// ROS
#include "pioneer_modules/sonar.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace pioneer_modules
{

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using std::placeholders::_2;

void Sonar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::weak_ptr<ArRobot> robot)
{
  // Declare and read parameters
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  robot_ = robot.lock();
  if (!robot_) {
    throw std::runtime_error("Unable to lock robot!");
  }

  // Declare and read parameters
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sonar_frame",
    rclcpp::ParameterValue("sonar_link"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the sonar frame of the robot"));
  node->get_parameter(plugin_name_ + ".sonar_frame", sonar_frame_);
  RCLCPP_INFO(logger_, "The parameter sonar_frame is set to: [%s]", sonar_frame_.c_str());

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".sonar_topic",
    rclcpp::ParameterValue("sonar"), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("The name of the sonar topic"));
  node->get_parameter(plugin_name_ + ".sonar_topic", sonar_topic_);
  RCLCPP_INFO(logger_, "The parameter sonar_topic is set to: [%s]", sonar_topic_.c_str());

  bool enable_sonar;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_sonar_at_startup",
    rclcpp::ParameterValue(true), rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Enable sonar at startup"));
  node->get_parameter(plugin_name_ + ".enable_sonar_at_startup", enable_sonar);
  RCLCPP_INFO(
    logger_, "The parameter enable_sonar_at_startup is set to: [%s]",
      enable_sonar ? "true" : "false");
  if (enable_sonar) {
    robot_->enableSonar();
  } else {
    robot_->disableSonar();
  }

  // Create ROS publishers
  sonar_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    sonar_topic_, rclcpp::SystemDefaultsQoS());

  // Create Aria subscribers
  sonar_callback_functor_ =
    std::make_unique<ArFunctorC<Sonar>>(this, &Sonar::sonarDataCallback);
  robot_->addSensorInterpTask("sonar", 100, sonar_callback_functor_.get());

  // Callback for monitor changes in parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Sonar::dynamicParametersCallback, this, _1));
}

void Sonar::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type pioneer_module::Sonar", plugin_name_.c_str());
  sonar_pub_.reset();
}

void Sonar::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type pioneer_module::Sonar", plugin_name_.c_str());
  sonar_pub_->on_activate();
}

void Sonar::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type pioneer_module::Sonar", plugin_name_.c_str());
  sonar_pub_->on_deactivate();
  robot_->disableSonar();
}

rcl_interfaces::msg::SetParametersResult Sonar::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_BOOL) {
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == plugin_name_ + ".sonar_frame") {
        sonar_frame_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter sonar_frame is set to: [%s]", sonar_frame_.c_str());
      } else if (name == plugin_name_ + ".sonar_topic") {
        sonar_topic_ = parameter.as_string();
        RCLCPP_INFO(logger_, "The parameter sonar_topic is set to: [%s]", sonar_topic_.c_str());
      }
    }
  }

  result.successful = true;
  return result;
}

void Sonar::sonarDataCallback()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = sonar_frame_;
  cloud.header.stamp = clock_->now();

  // Define the fields for the PointCloud2 message
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(robot_->getNumSonar());

  // Fill the PointCloud2 data
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (int i = 0; i < robot_->getNumSonar(); ++i) {
    ArSensorReading *reading = robot_->getSonarReading(i);
    if (!reading) {
      continue;
    }

    // The sonar reading is in mm, convert to meters
    *iter_x = reading->getLocalX() / 1000.0;
    *iter_y = reading->getLocalY() / 1000.0;
    *iter_z = 0.0;

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  // Publish the sonar data
  sonar_pub_->publish(cloud);
}

}  // namespace pioneer_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(pioneer_modules::Sonar, pioneer_core::Module)
