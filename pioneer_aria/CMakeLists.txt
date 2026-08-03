cmake_minimum_required(VERSION 3.5)
project(pioneer_aria)

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pioneer_core REQUIRED)
find_package(pioneer_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

pioneer_default_build_settings()

# ##########
# # Build ##
# ##########
include_aria_directories()

set(library_name aria_core)
set(executable_name aria_framework)

# Add library
add_library(${library_name} SHARED src/aria_framework.cpp)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name}
  PUBLIC
  ${diagnostic_msgs_TARGETS}
  nav2_util::nav2_util_core
  pioneer_core::pioneer_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  PRIVATE
  rclcpp_components::component
)
target_link_aria_libraries(${library_name})

# Add executable
add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name} PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

rclcpp_components_register_nodes(${library_name} "pioneer_aria::AriaFramework")

# ############
# # Install ##
# ############
install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  diagnostic_msgs
  nav2_util
  pioneer_core
  pluginlib
  rclcpp
  rclcpp_components
)
ament_export_targets(${library_name})
ament_package()