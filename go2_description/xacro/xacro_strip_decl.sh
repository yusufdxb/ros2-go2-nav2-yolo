#!/bin/bash
# Run xacro and strip the <?xml?> declaration line.
# gazebo_ros2_control in Humble fails when the URDF param contains this.
xacro "$@" | sed '/<?xml/d'
