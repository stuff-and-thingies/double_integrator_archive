#!/usr/bin/env zsh
set -e

cd ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
