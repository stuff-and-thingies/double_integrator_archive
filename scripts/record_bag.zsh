#!/usr/bin/env zsh
set -e

cd ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

mkdir -p bags


ros2 bag record -s mcap -a -o bags/fake_drone_sim
