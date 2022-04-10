#!/bin/sh

echo "Deploying a factorybot in a warehouse environment"
gnome-terminal --tab --title 'gazebo' -- sh -c "roslaunch factorybot sim_world.launch factorybot_type:=agv factorybot_name:=agv"
sleep 5

echo "Opening rviz to observe the map"
gnome-terminal --tab --title 'rviz' -- sh -c "roslaunch factorybot rviz_visualize_agv_urdf.launch rviz_config:=gmapping_agv.rviz" 
sleep 5

echo "Starting gmapping to create an occupancy grid map"
gnome-terminal --tab --title 'gmapping' -- sh -c  "roslaunch factorybot agv_gmapping.launch agv_name:=agv" 
sleep 5

echo "Starting the teleop terminal to move robot manually "
gnome-terminal --tab --title 'teleop' -- sh -c  "roslaunch factorybot keyboard_teleop.launch factorybot_name:=agv"