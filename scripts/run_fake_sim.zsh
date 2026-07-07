#!/usr/bin/env zsh

set -e

cd ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

ros2 launch fake_drone_sim fake_drone_sim.launch.py
