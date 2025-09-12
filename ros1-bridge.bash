#!/bin/bash

if ! grep -q "Ubuntu 20.04" /etc/os-release; then
  echo "available only on Ubuntu 20.04"
  exit 1
fi

gnome-terminal \
  --tab --title="ROS1 core" -e "bash -c 'source /opt/ros/noetic/setup.bash; roscore; exec bash'" \
  --tab --title="ROS1 to ROS2 bridge" -e "bash -c
      'sleep 3;
      source /opt/ros/noetic/setup.bash;
      source /opt/ros/foxy/setup.bash;
      ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --verbose;
      exec bash'" \

# ROS1: rostopic pub -r 1 /chatter std_msgs/String "data: 'from ROS1'"
# ROS2: ros2 topic echo /chatter
