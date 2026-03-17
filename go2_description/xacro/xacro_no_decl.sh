#!/bin/bash
# Wrapper: run xacro but strip the <?xml ...?> declaration
# gazebo_ros2_control in Humble fails to parse the URDF when it contains
# the XML declaration because it passes it via --param CLI arg.
xacro "$@" | grep -v '^<?xml'
