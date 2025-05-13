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

#ifndef PIONEER_CORE__ARIA_LOGGER_HPP_
#define PIONEER_CORE__ARIA_LOGGER_HPP_

#include "Aria/ArLog.h"
#include "rclcpp/logger.hpp"

namespace pioneer_core
{

/**
 * @class pioneer_core::AriaLogger
 * @brief Class for Aria log sinks. Redirect Aria logging to RCLCPP logging.
 */
class AriaLogger : public ArFunctor2<const char *, ArLog::LogLevel>
{
public:
  /**
   * @brief Construct a new Logger object
   *
   * @param logger RCLCPP logger to redirect Aria logging.
   * @param level Aria log level to determine the severity of the logs.
   */
  explicit AriaLogger(rclcpp::Logger logger)
  : logger_(logger){}

  /**
   * @brief Invoke without parameters.
   */
  void invoke() override{}

  /**
   * @brief Invoke without parameters.
   *
   * @param message The log message from Aria.
   */
  void invoke(const char *message) override{
    RCLCPP_INFO(logger_, message);
  }

  /**
   * @brief Redirect Aria logs to RCLCPP logs based on the log level.
   *
   * @param message The log message from Aria.
   */
  void invoke(const char *message, ArLog::LogLevel level) override
  {
    // Convert Aria log level to RCLCPP log level
    switch (level) {
      case ArLog::Terse:
        RCLCPP_WARN(logger_, message); 
        break;
      case ArLog::Normal:
        RCLCPP_INFO(logger_, message);
        break;
      case ArLog::Verbose:
        RCLCPP_DEBUG(logger_, message);
        break;
      default:
        RCLCPP_INFO(logger_, message);
        break;
    }
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace pioneer_core

#endif  // PIONEER_CORE__ARIA_LOGGER_HPP_