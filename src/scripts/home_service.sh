#!/bin/sh

xterm -e "roslaunch turtlebot3_gazebo turtlebot3_world.launch" &

sleep 5

xterm -e "rosrun rviz_launchers set_robot_initial_position" &

sleep 5

xterm -e "roslaunch rviz_launchers view_navigation.launch" &

sleep 5

xterm -e "rosnode kill set_robot_initial_position" &

sleep 5

xterm -e "rosrun add_markers add_markers" &

sleep 5

xterm -e "rosrun pick_objects pick_objects" 
