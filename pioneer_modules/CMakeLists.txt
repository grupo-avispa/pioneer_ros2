cmake_minimum_required(VERSION 3.5)
project(pioneer_modules)

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pioneer_msgs REQUIRED)
find_package(pioneer_core REQUIRED)
find_package(pioneer_common REQUIRED)
find_package(pluginlib REQUIRED)

pioneer_default_build_settings()

# ##########
# # Build ##
# ##########
include_aria_directories()

add_library(pioneer_charger SHARED src/charger.cpp)
target_include_directories(pioneer_charger PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pioneer_charger
  PUBLIC
  rclcpp::rclcpp
  pioneer_core::pioneer_core
  ${pioneer_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  PRIVATE
  pluginlib::pluginlib
)
target_link_aria_libraries(pioneer_charger)

add_library(pioneer_drive SHARED src/drive.cpp)
target_include_directories(pioneer_drive PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pioneer_drive
  PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  pioneer_core::pioneer_core
  ${pioneer_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  PRIVATE
  pluginlib::pluginlib
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_link_aria_libraries(pioneer_drive)

add_library(pioneer_sonar SHARED src/sonar.cpp)
target_include_directories(pioneer_sonar PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pioneer_sonar
  PUBLIC
  rclcpp::rclcpp
  pioneer_core::pioneer_core
  ${sensor_msgs_TARGETS}
  PRIVATE
  pluginlib::pluginlib
)
target_link_aria_libraries(pioneer_sonar)

# ############
# # Install ##
# ############
install(TARGETS
  pioneer_charger
  pioneer_drive
  pioneer_sonar
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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

  ament_add_gtest(test_drive test/test_drive.cpp)
  target_link_libraries(test_drive pioneer_drive)

  ament_add_gtest(test_charger test/test_charger.cpp)
  target_link_libraries(test_charger pioneer_charger)
endif()

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(
  pioneer_charger
  pioneer_drive
  pioneer_sonar
)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  pioneer_core
  pioneer_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(pioneer_core modules_plugin.xml)

ament_export_targets(${PROJECT_NAME})
ament_package()