# pioneer_modules

## Overview

This package contains the implementations of the Pioneer modules that interface with the Aria framework, some of which expose the robot's functionality to ROS 2 via topics and services. Currently, the following `modules` are supported:

* **Charger**: Monitors the state of the battery and the charging station, providing real-time updates on battery levels and charging status.

* **Drive**: Controls the robot's motors. This includes handling velocity commands, tracking odometry, managing bumpers, and more.

## Modules

### Charger

The Charger module monitors the state of the battery and the charging station.

#### Published Topics

* **`battery_state`** ([sensor_msgs/BatteryState])

	Publishes the current state of the battery.

### Drive

The Drive module controls the robot's motors, handling velocity commands, odometry, bumpers, and more.

#### Subscribed Topics

* **`cmd_vel`** ([geometry_msgs/TwistStamped])

	Subscribes to the velocity of the robot sent to the motor controller.

#### Published Topics

* **`odom`** ([nav_msgs/Odometry])

	Publishes the robot's odometry. This is also published as a TF between `/odom` and `/base_footprint`.

* **`bumper`** ([pioneer_msgs/BumperState])

	Publishes the current state of the robot's bumper.

#### Services

* **`enable_motors`** ([pioneer_msgs/EnableMotors])

	This service takes a `std_msgs::Bool enabled` in the request, and gives an empty response. Disabling the motors is the same as placing the robot into "Free Run" mode from the status display.

#### Parameters

* **`robot_base_frame`** (string, default: base_link)

	Specifies the name of the base frame of the robot.

* **`odom_frame`** (string, default: odom)

	Specifies the name of the odometry frame when publishing the TF.

* **`odom_topic`** (string, default: odom)

	Specifies the name of the odometry topic.

* **`publish_tf`** (bool, default: true)

	This parameter should be set to true to publish the TF between `odom_frame` and `robot_base_frame`.

* **`velocity_timeout`** (int, default: 20)

	The interval in milliseconds to check for new velocity commands. If no command is received within this time, the robot will stop.

### Sonar

The Sonar module controls the robot's sonar sensors, providing real-time updates on the distance to obstacles.

#### Published Topics

* **`sonar`** ([sensor_msgs/PointCloud2])

	Publishes the sonar data as a point cloud.

#### Parameters

* **`sonar_frame`** (string, default: sonar_link)

	Specifies the name of the sonar frame.

* **`sonar_topic`** (string, default: sonar)

	Specifies the name of the sonar topic.


[nav_msgs/Odometry]: http://docs.ros2.org/jazzy/api/nav_msgs/msg/Odometry.html
[geometry_msgs/TwistStamped]: http://docs.ros2.org/jazzy/api/geometry_msgs/msg/TwistStamped.html
[sensor_msgs/BatteryState]: https://docs.ros2.org/jazzy/api/sensor_msgs/msg/BatteryState.html
[sensor_msgs/PointCloud2]: http://docs.ros2.org/jazzy/api/sensor_msgs/msg/PointCloud2.html
[pioneer_msgs/BumperState]: ../pioneer_msgs/msg/BumperState.msg
[pioneer_msgs/EnableMotors]: ../pioneer_msgs/srv/EnableMotors.srv
