#!/usr/bin/env bash
# GO2 simulation environment — uses loopback, no real robot hardware needed

unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION

export DISPLAY=:1

# Software rendering for gzclient — hardware GPU causes boost::shared_ptr<Camera>
# assertion crash (gzclient rendering::Camera race condition on this MacBook Pro).
# LIBGL_ALWAYS_SOFTWARE=1 bypasses the crash entirely.
# NOTE: DRI_PRIME=0 = AMD (card1/renderD128), DRI_PRIME=1 = Intel (card2/renderD129)
export DRI_PRIME=0
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_DRI3_DISABLE=1      # prevents gzclient rendering::Camera crash with llvmpipe
export OGRE_RTT_MODE=Copy
export __GL_THREADED_OPTIMIZATIONS=0

source /opt/ros/humble/setup.bash
source ~/go2_yolo_ws/install/setup.bash

export ROS_DOMAIN_ID=0

echo "=== GO2 SIM ENV READY ==="
echo "RMW: $(printenv RMW_IMPLEMENTATION || echo 'default (fastrtps)')"
echo "CYCLONEDDS_URI: $(printenv CYCLONEDDS_URI || echo 'unset (loopback safe)')"
