#!/bin/bash
declare -a PIDS=()

test "x$DRONES" != "x" || DRONES=1;
test "x$BOTS" != "x" || BOTS=1;
test "x$RVIZ" != "x" || RVIZ=0;
test "x$DELAY" != "x" || DELAY=5;
test "x$CONTROLLERS" != "x" || CONTROLLERS=1;
test "x$MAP" != "x" || MAP=1;
test "x$TAGS" != "x" || TAGS=0;

mkdir -p logs;
echo Launch the world
roslaunch world simulator.launch >logs/simulator.log 2>&1 &
PIDS+=($!);
sleep ${DELAY};

if test $BOTS -ne 0;
then
  echo Bring in the turtle
  export ROBOT_INITIAL_POSE="-x -0.5 -y -0.5";
  roslaunch wasp_g1_start start_turtlebot_sim.launch >logs/turtle.log 2>&1 &
  PIDS+=($!);
  sleep ${DELAY};
fi;

if test $DRONES -ne 0;
then
  echo Add drones
  roslaunch world ardrone_sim.launch >logs/drone.log 2>&1 &
  PIDS+=($!);
  sleep ${DELAY};
  if test $CONTROLLERS -ne 0;
  then
    echo Add drone controllers
    roslaunch world ardrone_sim_control.launch >logs/dronecontrol.log 2>&1 &
    PIDS+=($!);
  fi;
fi;

if test $MAP -ne 0;
then
  echo "Add map"
  roslaunch world sim_map.launch >logs/map.log 2>&1 &
  PIDS+=($!);
fi;

if test $RVIZ -ne 0;
then
  echo "RViz"
  roslaunch turtlebot_rviz_launchers view_navigation.launch >logs/rviz.log 2>&1 &
  PIDS+=($!);
fi;

# Starts a tag detecion through PC camera.
if test $TAGS -ne 0;
then
  echo "Tag detection";
  roslaunch world sim_tag_person_detect.launch >logs/person_detect.log 2>&1 &
  PIDS+=($!);
fi;

read -p "Press enter to exit";

echo "Killing of nodes";
kill ${PIDS[@]};
echo "Done";

