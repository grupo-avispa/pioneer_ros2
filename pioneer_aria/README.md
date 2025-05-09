# pioneer_aria

## Overview

The `pioneer_aria` package serves as the primary interface between the Aria / AriaCoda framework and MobileRobots Pioneer robots. This package contains the main lifecycle node responsible for initiating and terminating the Aria framework, and providing a ROS 2 interface for the robot's modules.

## Usage

Launch the `pioneer_aria` node using the following command:

```bash
ros2 launch pioneer_aria aria_launch.py
```

## Nodes

### aria_framework

A lifecycle node that interfaces with the Aria framework.

#### Parameters

* **`module_plugins`** (string array, default: "")

	Specifies the modules to be loaded.
