#!/usr/bin/env bash

roslaunch motion_planning turtlebot3_house.launch model_name:=R1 world_file:=`rospack find turtlebot3_gazebo`/worlds/turtlebot3_house.world &

echo "Please hit ENTER to run RViz."
read enter

roslaunch motion_planning turtlebot3_navigation.launch map_file:=`rospack find motion_planning`/maps/map.yaml &

echo "Please hit ENTER to run SpawnRobot."
read enter

roslaunch motion_planning SpawnRobots.launch model_name1:=R2 x1:=-1.0 y1:=1.0 z1:=0.0 Y1:=3.14 &

echo "Please hit ENTER to run ModelStateSeparator."
read enter

rosrun motion_planning ModelStateSeparator -m R1 &

echo "Please hit ENTER to run ModelStatePublisher."
read enter

rosrun motion_planning ModelStatePublisher &

echo "Please hit ENTER to run ROSConnect serverSide."
read enter

roslaunch ros_web_client serverside.launch

