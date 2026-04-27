# pioneer_bringup

## Overview

This package contains the launch files to bring up the four-wheel skid-steer Pioneer 3AT robot.

## Usage

Just launch the file according to your needs.

For example:

	ros2 launch pioneer_bringup robot.launch.py

## Systemd service

To launch the robot at startup, you can install the following systemd service:

```bash
sudo mkdir -p /etc/robot_bringup && sudo cp local.env /etc/robot_bringup/
sudo mkdir -p /etc/nav2_bringup && sudo cp local.env /etc/nav2_bringup/
sudo cp robot_bringup.service /etc/systemd/system/ && sudo systemctl enable robot_bringup
sudo cp nav2_bringup.service /etc/systemd/system/ && sudo systemctl enable nav2_bringup
sudo systemctl start robot_bringup
sudo systemctl start nav2_bringup
```
