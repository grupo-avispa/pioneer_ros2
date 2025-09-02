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
#include <chrono>
#include <thread>

// ROS
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_ros_common/node_utils.hpp"

#include "pioneer_aria/aria_framework.hpp"

namespace pioneer_aria
{

AriaFramework::AriaFramework(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("pioneer_aria", "", options),
  connected_(false),
  module_loader_("pioneer_core", "pioneer_core::Module"),
  default_ids_{"drive"},
  default_types_{"pioneer_modules::Drive"}
{
  RCLCPP_INFO(get_logger(), "Creating Aria framework");

  // Redirect Aria logger
  aria_logger_ = std::make_shared<pioneer_core::AriaLogger>(this->get_logger());
  ArLog::setFunctor(aria_logger_.get());

  robot_ = std::make_shared<ArRobot>();
  args_ = std::make_unique<ArArgumentBuilder>();
  arg_parser_ = std::make_unique<ArArgumentParser>(args_.get());
}

AriaFramework::~AriaFramework()
{
  modules_.clear();

  robot_.reset();
  args_.reset();
  arg_parser_.reset();
  connector_.reset();
  timer_.reset();

  ArLog::clearFunctor();
}

nav2::CallbackReturn AriaFramework::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring Aria framework interface");

  // Adds any arguments given in /etc/Aria.args.
  // Useful on robots with unusual serial port or baud rate (e.g. Pioneer lx)
  arg_parser_->loadDefaultArguments();

  // Set the serial port and baudrate
  std::string serial_port;
  nav2::declare_parameter_if_not_declared(
    this, "serial_port", rclcpp::ParameterValue("/dev/ttyUSB0"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Serial port to connect to the robot"));
  this->get_parameter("serial_port", serial_port);

  // If serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos) {
    args_->add("-remoteHost");
    args_->add(serial_port.substr(0, colon_pos).c_str());
    args_->add("-remoteRobotTcpPort");
    args_->add(serial_port.substr(colon_pos + 1).c_str());
  } else {
    args_->add("-robotPort %s", serial_port.c_str());
  }

  nav2::declare_parameter_if_not_declared(
    this, "serial_baudrate", rclcpp::ParameterValue(9600),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Serial baudrate to connect to the robot"));
  int serial_baudrate = 0;
  this->get_parameter("serial_baudrate", serial_baudrate);

  if (serial_baudrate != 0) {
    args_->add("-robotBaud %d", serial_baudrate);
  }

  // Setup the robot
  connector_ = std::make_unique<ArRobotConnector>(arg_parser_.get(), robot_.get());
  if (!connector_->setupRobot()) {
    RCLCPP_FATAL(get_logger(), "Could not setup the robot. Check the serial port.");
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  nav2::declare_parameter_if_not_declared(
    this, "module_plugins",
    rclcpp::ParameterValue(default_ids_),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("List of the modules exposed by the node"));
  this->get_parameter("module_plugins", module_ids_);

  // Print the list of modules
  for (size_t i = 0; i != module_ids_.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Module: %s", module_ids_[i].c_str());
  }

  for (size_t i = 0; i != default_ids_.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Default module: %s", default_ids_[i].c_str());
  }

  if (module_ids_ == default_ids_) {
    for (size_t i = 0; i != default_ids_.size(); ++i) {
      nav2::declare_parameter_if_not_declared(
        this, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]),
        rcl_interfaces::msg::ParameterDescriptor()
        .set__description("Type of the module"));
    }
  }

  module_types_.resize(module_ids_.size());

  // Create Aria modules
  for (size_t i = 0; i != module_ids_.size(); i++) {
    try {
      module_types_[i] = nav2::get_plugin_type_param(node, module_ids_[i]);
      pioneer_core::Module::Ptr module =
        module_loader_.createUniqueInstance(module_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created module : %s of type %s",
        module_ids_[i].c_str(), module_types_[i].c_str());
      module->configure(node, module_ids_[i], robot_);
      modules_.insert({module_ids_[i], module});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create module. Exception: %s", ex.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != module_ids_.size(); i++) {
    module_ids_concat_ += module_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(), "Aria framework has %s modules available.", module_ids_concat_.c_str());

  // Create a publisher for diagnostics
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn AriaFramework::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Connect to the robot
  if (!connector_->connectRobot()) {
    RCLCPP_ERROR(get_logger(), "Could not connect to the robot.");
    on_deactivate(state);
    return nav2::CallbackReturn::FAILURE;
  }
  connected_ = true;

  // Activate the modules
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    try {
      it->second->activate();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to activate module. Exception: %s", ex.what());
      on_deactivate(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  // Run ArRobot background processing thread
  robot_->runAsync(true);

  // Create a timer to publish diagnostics
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), [this]() {
      diag_pub_->publish(createDiagnostics());
    });

  // Create bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn AriaFramework::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    it->second->deactivate();
  }

  // Stop the robot
  if (robot_->isRunning()) {
    robot_->stopRunning();
    robot_->waitForRunExit();
  }

  // Destroy bond connection
  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn AriaFramework::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  ModuleMap::iterator it;
  for (it = modules_.begin(); it != modules_.end(); ++it) {
    it->second->cleanup();
  }
  modules_.clear();

  diag_pub_.reset();
  timer_.reset();

  // Disconnect the robot
  if (connected_) {
    connector_->disconnectAll();
    connected_ = false;
  }


  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn AriaFramework::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2::CallbackReturn::SUCCESS;
}

diagnostic_msgs::msg::DiagnosticArray AriaFramework::createDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Aria framework";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "Aria framework is running";
  msg.status.push_back(status);
  return msg;
}

}  // namespace pioneer_aria

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pioneer_aria::AriaFramework)
