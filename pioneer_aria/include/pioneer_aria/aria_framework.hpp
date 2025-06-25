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

#ifndef PIONEER_ARIA__ARIA_FRAMEWORK_HPP_
#define PIONEER_ARIA__ARIA_FRAMEWORK_HPP_

// ARIA
#include <Aria/ArArgumentBuilder.h>
#include <Aria/ArArgumentParser.h>
#include <Aria/ArRobotConnector.h>
#include <Aria/ArRobot.h>

// C++
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

// Pioneer
#include "pioneer_core/aria_logger.hpp"
#include "pioneer_core/module.hpp"

namespace pioneer_aria
{

/**
 * @class pioneer_aria::AriaFramework
 * @brief Main class that handles the AriaCoda framework and the modules.
 *
 */
class AriaFramework : public nav2::LifecycleNode
{
public:
  using ModuleMap = std::unordered_map<std::string, pioneer_core::Module::Ptr>;

  /**
   * @brief Construct a new Aria Framework object
   *
   * @param options Node options
   */
  explicit AriaFramework(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Aria Framework object
   *
   */
  ~AriaFramework();

protected:
  /**
   * @brief Configures the modules plugin.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize module
   * plugin
   */
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates the modules.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates the modules.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Create a diagnostics message.
   */
  diagnostic_msgs::msg::DiagnosticArray createDiagnostics();

  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Aria connector, robot and argument parser
  std::shared_ptr<ArRobot> robot_;
  std::unique_ptr<ArRobotConnector> connector_;
  std::unique_ptr<ArArgumentBuilder> args_;
  std::unique_ptr<ArArgumentParser> arg_parser_;
  std::shared_ptr<pioneer_core::AriaLogger> aria_logger_;
  bool connected_;

  // Module Plugins
  pluginlib::ClassLoader<pioneer_core::Module> module_loader_;
  ModuleMap modules_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> module_ids_;
  std::vector<std::string> module_types_;
  std::string module_ids_concat_;
};

}  // namespace pioneer_aria

#endif  // PIONEER_ARIA__ARIA_FRAMEWORK_HPP_
