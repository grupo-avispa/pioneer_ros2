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

#ifndef PIONEER_MODULES__SONAR_HPP_
#define PIONEER_MODULES__SONAR_HPP_

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
#include "sensor_msgs/msg/point_cloud2.hpp"

// PIONEER
#include "pioneer_core/module.hpp"

namespace pioneer_modules
{

/**
 * @class pioneer_modules::Sonar
 * @brief Module for interfacing all related to sonar.
 *
 */
class Sonar : public pioneer_core::Module
{
public:
  /**
   * @brief Construct for pioneer_modules::Sonar
   */
  Sonar() = default;

  /**
   * @brief Destructor for pioneer_modules::Sonar
   */
  ~Sonar() override = default;

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
   * @brief Callback executed when the sonar data is received.
   */
  void sonarDataCallback();

  /**
   * @brief Callback executed when a parameter change is detected.
   * @param event ParameterEvent message.
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Plugin related
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("Sonar")};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Aria related
  std::unique_ptr<ArFunctorC<Sonar>> sonar_callback_functor_;

  // ROS Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>>
  sonar_pub_;

  std::string sonar_frame_, sonar_topic_;
};

}  // namespace pioneer_modules

#endif  // PIONEER_MODULES__SONAR_HPP_
