#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
cd /ubuntu/ubuntu/auto_ws/ && catkin_make -DCMAKE_BUILD_TYPE=Release