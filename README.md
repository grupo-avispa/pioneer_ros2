# pioneer_ros2

![ROS2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/pioneer_ros2)
[![Build](https://github.com/grupo-avispa/pioneer_ros2/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/grupo-avispa/pioneer/actions/workflows/build.yml)

## Overview

`pioneer_ros2` is a ROS 2 stack designed for MobileRobots Pioneer robots that utilize the Aria / AriaCoda framework, including models such as Pioneer 2, Pioneer 2-DX, Pioneer 2-AT, Pioneer 3, Pioneer 3-DX, Pioneer 3-AT, etc. This stack comprises several packages, each serving a unique purpose:

 * [pioneer_common]: This package provides common functionalities for the pioneer stack.
 * [pioneer_core]: This package provides the abstract interface (virtual base classes) for the Pioneer Modules.
 * [pioneer_aria]: This is the main node that interfaces with the Aria framework.
 * [pioneer_modules]: This package contains the implementations of the Pioneer modules such as `drive`, `charger`, etc.
 * [pioneer_msgs]: This package contains messages and services related to the MobileRobots Pioneer robot base.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (middleware for robotics)
- [AriaCoda](https://github.com/grupo-avispa/AriaCoda) (MobileRobots Pioneer middleware for robotic applications)

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using the following command:
```bash
cd colcon_workspace/src
git clone https://github.com/grupo-avispa/pioneer_ros2.git
cd ../
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
```

[pioneer_common]: /pioneer_common
[pioneer_core]: /pioneer_core
[pioneer_aria]: /pioneer_aria
[pioneer_modules]: /pioneer_modules
[pioneer_msgs]: /pioneer_msgs
