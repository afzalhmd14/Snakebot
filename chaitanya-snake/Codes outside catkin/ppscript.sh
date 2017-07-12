#!/bin/bash

# A bash script to continuously run training simulations

# Contains paths to external files.

SIM_CNT=1

while [ 0 -le 1000 ]
do
	echo Starting Simulation $SIM_CNT

	gnome-terminal -e '/bin/bash -c "cd ~/catkin_ws && roslaunch snake_robot control_gazebo.launch gui:=false"' &
	gnome-terminal -e '/bin/bash -c "cd ~/catkin_ws && sleep 10s && rosrun path_planning path_planning_node && pkill roslaunch"'
	wait

	echo Simulation $SIM_CNT Completed
	echo Creating Random World

	gnome-terminal -e '/bin/bash -c "cd ~/Desktop/Work/BTP/Path_Planning && python create_world_paths.py"'

	echo Random World Created
	let SIM_CNT++
done