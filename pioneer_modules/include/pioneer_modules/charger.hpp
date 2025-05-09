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

#ifndef PIONEER_MODULES__CHARGER_HPP_
#define PIONEER_MODULES__CHARGER_HPP_

// Aria
#include <Aria/ArRobot.h>

// C++
#include <memory>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// PIONEER
#include "pioneer_core/module.hpp"

namespace pioneer_modules
{

/**
 * @class pioneer_modules::Charger
 * @brief Module for communicating with the charger and battery.
 *
 */
class Charger : public pioneer_core::Module
{
public:
  /**
   * @brief Construct for pioneer_modules::Charger
   */
  Charger() = default;

  /**
   * @brief Destructor for pioneer_modules::Charger
   */
  ~Charger() override = default;

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
   * @brief Callback for battery data.
   */
  void batteryDataCallback();

  // Plugin related
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("Charger")};

  std::unique_ptr<ArFunctorC<Charger>> battery_callback_functor_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>>
  battery_pub_;
};

}  // namespace pioneer_modules

#endif  // PIONEER_MODULES__CHARGER_HPP_
