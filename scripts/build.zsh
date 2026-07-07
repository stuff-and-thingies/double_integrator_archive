#!/usr/bin/env zsh

set -e

cd ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 

source ./install/setup.zsh
echo -e "\nsourced ./install/setup.zsh"

cd -

set +e
