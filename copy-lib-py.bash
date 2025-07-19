#!/bin/bash

# ROS 1
rm -rf lib-py/ros1
cp -r /opt/ros/noetic/lib/python3/dist-packages lib-py/ros1

# ROS 2
rm -rf lib-py/ros2
cp -r /opt/ros/foxy/lib/python3.8/site-packages lib-py/ros2

# prune egg-info files
rm -r lib-py/**/*egg-info
