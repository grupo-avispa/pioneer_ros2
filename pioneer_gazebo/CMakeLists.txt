cmake_minimum_required(VERSION 3.5)
project(pioneer_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY configs DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
