#!/bin/bash
ros2 topic pub -r 5 /cmd_vel_nav geometry_msgs/msg/Twist "{linear: {x: 0.00, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"