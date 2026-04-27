# pioneer_aria

## Overview

The `pioneer_aria` package serves as the primary interface between the Aria / AriaCoda framework and MobileRobots Pioneer robots. This package contains the main lifecycle node responsible for initiating and terminating the Aria framework, and providing a ROS 2 interface for the robot's modules.

## Usage

Launch the `pioneer_aria` node using the following command:

```bash
ros2 launch pioneer_aria aria_launch.py
```

## Setup udev rules

This rules are necessary to allow the Pioneer robot to be accessed by the Aria framework. This rules should be installed during the Aria software installation. However, if you need to install them manually, follow the instructions below.

```bash
cd pioneer_ros2/pioneer_aria
sudo cp rules/99-ARIA.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Nodes

### aria_framework

A lifecycle node that interfaces with the Aria framework.

#### Parameters

* **`module_plugins`** (string array, default: "")

	Specifies the modules to be loaded.
