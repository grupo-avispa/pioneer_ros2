# pioneer_gazebo

## Overview

This package contains the models to simulate a Pioneer robot on Gazebo environment. Includes a world similar to a lab.

![Gazebo](doc/gazebo.png)

## Usage

To launch the simulation, you can use the launch files inside the `launch` folder:

	ros2 launch pioneer_gazebo robot.launch.py

Then, you can launch the navigation stack with a map file as follows:

	ros2 launch pioneer_nav bringup.launch.py map_name:='lab_with_corridor' use_sim_time:=True

Lastly, you can launch the RViz visualization:

	ros2 launch pioneer_nav rviz.launch.py
