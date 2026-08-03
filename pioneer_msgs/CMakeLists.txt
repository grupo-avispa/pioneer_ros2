cmake_minimum_required(VERSION 3.5)
project(pioneer_msgs)

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pioneer_common REQUIRED)

pioneer_default_build_settings()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################
set(msg_files
  "msg/BumperState.msg"
)
set(srv_files
  "srv/EnableMotors.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

# ##################################
# # ament specific configuration ##
# ##################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()