#!/usr/bin/env bash

roslaunch motion_planning turtlebot3_house.launch model_name:=R2 x_pos:=-1.0 y_pos:=1.0 z_pos:=0.0 yaw:=3.14 world_file:=`rospack find turtlebot3_gazebo`/worlds/turtlebot3_house.world &

echo "Please hit ENTER to run RViz."
read enter

roslaunch motion_planning turtlebot3_navigation.launch map_file:=`rospack find motion_planning`/maps/map.yaml initial_pose_x:=-1.0 initial_pose_y:=1.0 initial_pose_a:=3.14 &

echo "Please hit ENTER to run SpawnRobot."
read enter

roslaunch motion_planning SpawnRobots.launch model_name1:=R1 x1:=-3.0 y1:=1.0 z1:=0.0 Y1:=0.0 &

echo "Please hit ENTER to run ModelStateSeparator."
read enter

rosrun motion_planning ModelStateSeparator -m R2 &

echo "Please hit ENTER to run ModelStatePublisher."
read enter

rosrun motion_planning ModelStatePublisher &

echo "Please hit ENTER to run ROSConnect clientSide."
read enter

roslaunch ros_web_client clientside.launch url:="ws://192.168.18.139:8080/ws"
