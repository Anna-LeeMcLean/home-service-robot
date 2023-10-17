#!/bin/sh

xterm -e "roslaunch turtlebot3_gazebo turtlebot3_world.launch" &

sleep 5

xterm -e "roslaunch turtlebot3_navigation turtlebot3_navigation.launch" &

