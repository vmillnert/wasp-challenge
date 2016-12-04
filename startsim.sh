#!/bin/bash
declare -a PIDS=()


if [ ! $DELAY ];
then
  DELAY=5;
fi;

mkdir -p logs;
echo Launch the world
roslaunch world simulator.launch >logs/simulator.log 2>&1 &
PIDS+=($!);
sleep ${DELAY};

rosparam set /available_drones ["drone"]
rosparam set /available_turtlebots ["turtle"]

if [ ! $NOBOT ];
then
  echo Bring in the turtle
  export ROBOT_INITIAL_POSE="-x -0.5 -y -0.5";
  roslaunch wasp_g1_start start_turtlebot_sim.launch >logs/turtle.log 2>&1 &
  PIDS+=($!);
  sleep ${DELAY};
fi;

if [ ! $NODRONE ];
then
  echo Add drone
  roslaunch world ardrone_sim.launch >logs/drone.log 2>&1 &
  PIDS+=($!);
  sleep ${DELAY};
  DRONESIM= rosrun bebop_controller BebopActionServer.py > logs/dronecontroller.log 2>&1 &
  PIDS+=($!);
fi;

echo "Add map"
roslaunch world sim_map.launch >logs/map.log 2>&1 &
PIDS+=($!);

if [ ! $NORVIZ ];
then
  echo "RViz"
  roslaunch turtlebot_rviz_launchers view_navigation.launch >logs/rviz.log 2>&1 &
fi;

read -p "Press enter to exit";

echo "Killing of nodes";
kill ${PIDS[@]};
echo "Done";

