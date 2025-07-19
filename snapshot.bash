#!/bin/bash

rm -rf snapshot
mkdir -p snapshot/ros1
mkdir -p snapshot/ros2

# ROS 1
export ROS_ROOT=/opt/ros/noetic
cp -r ${ROS_ROOT}/lib/python3/dist-packages snapshot/ros1/lib-py
cp -r ${ROS_ROOT}/share snapshot/ros1/share

# ROS 2
export ROS_ROOT=/opt/ros/foxy
cp -r ${ROS_ROOT}/lib/python3.8/site-packages snapshot/ros2/lib-py
cp -r ${ROS_ROOT}/share snapshot/ros2/share

# prune egg-info files
rm -r snapshot/*/lib-py/*egg-info
