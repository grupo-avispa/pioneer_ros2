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

// C++
#include <limits>

#include "pioneer_modules/charger.hpp"

namespace pioneer_modules
{

using std::placeholders::_1;
using std::placeholders::_2;

void Charger::configure(
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

  // Create ROS publishers
  battery_pub_ = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery", rclcpp::SystemDefaultsQoS());

  // Create Aria subscribers
  battery_callback_functor_ =
    std::make_unique<ArFunctorC<Charger>>(this, &Charger::batteryDataCallback);
  robot_->addSensorInterpTask("batterState", 100, battery_callback_functor_.get());

  RCLCPP_INFO(logger_, "Configured module : %s", plugin_name_.c_str());
}

void Charger::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up module : %s of type pioneer_module::Charger", plugin_name_.c_str());
  battery_pub_.reset();
}

void Charger::activate()
{
  RCLCPP_INFO(
    logger_, "Activating module : %s of type pioneer_module::Charger", plugin_name_.c_str());
  battery_pub_->on_activate();
}

void Charger::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating module : %s of type pioneer_module::Charger", plugin_name_.c_str());
  battery_pub_->on_deactivate();
}

void Charger::batteryDataCallback()
{
  sensor_msgs::msg::BatteryState battery;

  battery.header.frame_id = "base_link";
  battery.header.stamp = clock_->now();
  battery.voltage = robot_->getRealBatteryVoltageNow();
  battery.temperature = robot_->hasTemperature() ?
    robot_->getTemperature() : std::numeric_limits<float>::quiet_NaN();
  battery.percentage = robot_->haveStateOfCharge() ? robot_->getStateOfCharge() / 100.0 :
    std::numeric_limits<float>::quiet_NaN();

  if (robot_->isChargerPowerGood()) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (battery.percentage == 1.0) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else if (robot_->getChargeState() == ArRobot::ChargeState::CHARGING_NOT) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (robot_->getChargeState() == ArRobot::ChargeState::CHARGING_UNKNOWN) {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  } else {
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }

  battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery.location = "Unknown";
  battery.serial_number = "Unknown";

  battery_pub_->publish(battery);
}

}  // namespace pioneer_modules

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(pioneer_modules::Charger, pioneer_core::Module)
