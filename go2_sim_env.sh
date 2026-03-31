#!/usr/bin/env bash
# GO2 simulation environment — uses loopback, no real robot hardware needed

unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION

export DISPLAY=:1

# Hardware GPU rendering (NVIDIA RTX 5070 on mewtwo).
# LIBGL_DRI3_DISABLE=1 is still needed for the depth camera OGRE sensor init.
export LIBGL_DRI3_DISABLE=1
export OGRE_RTT_MODE=Copy

source /opt/ros/humble/setup.bash
source ~/workspace/ros2-go2-nav2-yolo/install/setup.bash

export ROS_DOMAIN_ID=0

echo "=== GO2 SIM ENV READY ==="
echo "RMW: $(printenv RMW_IMPLEMENTATION || echo 'default (fastrtps)')"
echo "CYCLONEDDS_URI: $(printenv CYCLONEDDS_URI || echo 'unset (loopback safe)')"
