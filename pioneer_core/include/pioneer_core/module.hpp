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

#ifndef PIONEER_CORE__MODULE_HPP_
#define PIONEER_CORE__MODULE_HPP_

// Aria
#include <Aria/ArRobot.h>

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pioneer_core
{

/**
 * @class pioneer_core::Module
 * @brief Base class for all Pioneer modules (Drive, Laser, Sonar, Display, etc).
 */
class Module
{
public:
  using Ptr = std::shared_ptr<pioneer_core::Module>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Module() {}

  /**
   * @param parent pointer to user's node
   * @param name Name of the module
   * @param robot pointer to the ArRobot object
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::weak_ptr<ArRobot> robot) = 0;

  /**
   * @brief Method to cleanup resources.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active the module and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate the module and any threads involved in execution.
   */
  virtual void deactivate() = 0;

protected:
  /**
   * @brief Declares static ROS2 parameter and sets it to a given value if it was not already declared.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param default_value Parameter value to initialize with
   * @param parameter_descriptor Parameter descriptor (optional)
  */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, default_value, parameter_descriptor);
    }
  }

  std::shared_ptr<ArRobot> robot_;
};

}  // namespace pioneer_core

#endif  // PIONEER_CORE__MODULE_HPP_
