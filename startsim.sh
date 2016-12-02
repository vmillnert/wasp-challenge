#!/bin/bash
declare -a PIDS=()

mkdir -p logs;
echo Launch the world
roslaunch world simulator.launch >logs/simulator.log 2>&1 &
PIDS+=($!);
sleep 5;

rosparam set /available_drones ["drone"]
rosparam set /available_turtlebots ["turtle"]

if [ ! $NOBOT ];
then
  echo Bring in the turtle
  export ROBOT_INITIAL_POSE="-x -0.5 -y -0.5";
  roslaunch wasp_g1_start start_turtlebot_sim.launch >logs/turtle.log 2>&1 &
  PIDS+=($!);
  sleep 5;
fi;

if [ ! $NODRONE ];
then
  echo Add drone
  roslaunch world ardrone_sim.launch >logs/drone.log 2>&1 &
  PIDS+=($!);
  sleep 5;
fi;

echo "Add map"
roslaunch world sim_map.launch >logs/map.log 2>&1 &
PIDS+=($!);

echo "RViz"
roslaunch turtlebot_rviz_launchers view_navigation.launch

echo "Killing of nodes";
kill ${PIDS[@]};
echo "Done";

